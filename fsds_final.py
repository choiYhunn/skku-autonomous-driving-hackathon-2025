#!/usr/bin/env python3

# v11.1 수정본

import rospy
import numpy as np
from scipy.interpolate import splprep, splev 
from scipy.spatial.distance import cdist
# [NEW] 최적화를 위한 라이브러리 추가
from scipy.optimize import minimize

from fs_msgs.msg import Track, Cone, ControlCommand
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Point, TwistWithCovarianceStamped, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import tf

# ==============================================================================
# [TUNING] 하이퍼파라미터 설정
# ==============================================================================

# --- 경로 생성 (Mapping) ---
TRACK_MATCH_DIST = 12.0
PATH_SMOOTHING_FACTOR = 0.5 # Spline Smoothing
PATH_DENSITY = 10           # 점의 밀도 (높을수록 정밀하지만 연산량 증가)
MAX_CONE_CONNECT_DIST = 15.0

# --- [NEW] 레이싱 라인 최적화 (Racing Line Optimization) ---
# 시뮬레이터에서 튜닝한 값들을 여기에 적용합니다.
OPTIMIZE_RACING_LINE = False  # True: 최적화 적용, False: 단순 중점 주행
RACING_LINE_ALPHA = 500.0    # 곡률 민감도 (클수록 직선/곡선 구분 확실)
RACING_LINE_BETA = 300.0     # 중앙 복귀 강도 (클수록 직선에서 중앙 고수)
SAFETY_MARGIN = 0.2          # 콘과의 최소 거리 비율 (0.3 = 30%)

# --- 차량 제원 ---
WHEELBASE = 1.55

# --- 조향 제어 (Pure Pursuit) ---
LOOKAHEAD_MIN = 4.3
LOOKAHEAD_MAX = 10.0
LOOKAHEAD_GAIN = 0.1
STEERING_GAIN = 1.45

# --- 속도 제어 ---
MAX_SPEED = 12.0 # 시뮬레이션보다 약간 상향
MIN_SPEED = 5.0
CORNER_STIFFNESS = 3.0

K_ACCEL = 0.5
MAX_ACCEL = 0.9
K_BRAKE = 0.5
MAX_BRAKE = 0.9

# 목표 속도에 얼마나 빠르게 갈 것인가? (스무딩)
V_ALPHA = 0.05

K_STRAIGHT = 0.05  # 이 값 이하는 완전 직선으로 간주 (최고속도)
K_CORNER_MAX = 0.13 # 측정된 최대 곡률 (최저속도 지점)

MIRROR_STEERING = -1.0

# ==============================================================================

def cone_color(cone_type):
    if cone_type == Cone.BLUE:
        return ColorRGBA(0.0, 0.0, 1.0, 1.0)
    elif cone_type == Cone.YELLOW:
        return ColorRGBA(1.0, 1.0, 0.0, 1.0)
    elif cone_type == Cone.ORANGE_BIG or cone_type == Cone.ORANGE_SMALL:
        return ColorRGBA(1.0, 0.5, 0.0, 1.0)
    else:
        return ColorRGBA(1.0, 1.0, 1.0, 1.0)

class ControlNode:
    def __init__(self):
        # Data storage
        self.blue_cones = []
        self.yellow_cones = []
        self.mid_points = [] # 최종 경로 점들이 저장됨
        self.state = [0.0, 0.0, 0.0, 0.0] # x, y, theta, v

        # Visualization
        self.cone_markers = MarkerArray()
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Control States
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        self.prev_brake = 0.0
        self.prev_target_v = 0.0

        # Subscribers
        rospy.Subscriber("/fsds/testing_only/track", Track, self.track_callback)
        rospy.Subscriber("/fsds/testing_only/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/fsds/gss", TwistWithCovarianceStamped, self.speed_callback)

        # Publishers
        self.cmd_pub = rospy.Publisher("/fsds/control_command", ControlCommand, queue_size=10)
        self.midline_path_pub = rospy.Publisher("/midpoint_path", Path, queue_size=10)
        self.cones_pub = rospy.Publisher("/cones_markers", MarkerArray, queue_size=10)

        # Debug Publishers
        self.target_pub = rospy.Publisher("/debug/target_point", Marker, queue_size=10)
        self.blue_bound_pub = rospy.Publisher("/debug/blue_boundary", Marker, queue_size=10)
        self.yellow_bound_pub = rospy.Publisher("/debug/yellow_boundary", Marker, queue_size=10)
        self.text_info_pub = rospy.Publisher("/debug/car_info", Marker, queue_size=10)
        self.steering_arrow_pub = rospy.Publisher("/debug/steering_arrow", Marker, queue_size=10)
        # [NEW] 레이싱 라인 디버그용 (빨간색 경로)
        self.racing_line_pub = rospy.Publisher("/debug/racing_line", Path, queue_size=10)

    def track_to_markers(self, track_msg, frame_id="fsds/map"):
        markers = MarkerArray()
        for i, cone in enumerate(track_msg.track):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = rospy.Time(0)
            m.ns = "cones"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = cone.location
            m.pose.orientation.w = 1.0
            m.scale = Vector3(0.3, 0.3, 0.3)
            m.color = cone_color(cone.color)
            m.lifetime = rospy.Duration(0.5)
            markers.markers.append(m)
        return markers

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        _, _, yaw = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])

        self.state[0] = x
        self.state[1] = y
        self.state[2] = np.degrees(yaw)

        self.tf_broadcaster.sendTransform(
            (x, y, z),
            (qx, qy, qz, qw),
            rospy.Time.now(),
            "fsds/FSCar",
            "fsds/map"
        )

    def speed_callback(self, msg):
       xv = msg.twist.twist.linear.x
       yv = msg.twist.twist.linear.y
       self.state[3] = np.hypot(xv, yv)

    def track_callback(self, msg):
        self.cone_markers = self.track_to_markers(msg)
        self.blue_cones = [c for c in msg.track if c.color == Cone.BLUE]
        self.yellow_cones = [c for c in msg.track if c.color == Cone.YELLOW]
        # track_callback에서 경로 계산 호출 (odom 콜백보다 주기 느림 = 연산 부하 분산)
        self.calculate_path_independent()

    def publish_boundary_line(self, sorted_points, color_type):
        """ 정렬된 콘들을 잇는 선을 시각화 """
        marker = Marker()
        marker.header.frame_id = "fsds/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = f"{color_type}_boundary"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1 # 선 두께

        if color_type == "blue":
            marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        else:
            marker.color = ColorRGBA(1.0, 1.0, 0.0, 1.0)

        for pt in sorted_points:
            p = Point()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            marker.points.append(p)

        if color_type == "blue":
            self.blue_bound_pub.publish(marker)
        else:
            self.yellow_bound_pub.publish(marker)

    def publish_midpoints(self):
        path_msg = Path()
        path_msg.header.frame_id = "fsds/map" 
        path_msg.header.stamp = rospy.Time.now()

        for pt in self.mid_points:
            pose = PoseStamped()
            pose.header.frame_id = "fsds/map"
            pose.pose.position = pt
            pose.pose.orientation.w = 1.0 
            path_msg.poses.append(pose)

        self.midline_path_pub.publish(path_msg)

    def publish_debug_dashboard(self, target_v, throttle, brake, steering, max_curvature):
        """ 차량 위에 현재 상태를 텍스트와 화살표로 표시 """
        
        # 1. Text Info (HUD)
        text_marker = Marker()
        text_marker.header.frame_id = "fsds/FSCar" # 차 기준
        text_marker.ns = "dashboard"
        text_marker.id = 0
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.scale.z = 0.4 # 글자 크기
        text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
        text_marker.pose.position.z = 1.5 # 차 머리 위 1.5m

        cur_v = self.state[3]
        text_marker.text = (
            f"Speed: {cur_v:.1f} / {target_v:.1f} m/s\n"
            f"Thr: {throttle:.2f} | Brk: {brake:.2f}\n"
            f"Steer: {steering:.2f}\n"
            f"MaxCurv: {max_curvature:.3f}"
        )
        self.text_info_pub.publish(text_marker)

        # 2. Steering Arrow (조향 방향 시각화)
        arrow = Marker()
        arrow.header.frame_id = "fsds/FSCar"
        arrow.ns = "steering"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale = Vector3(0.1, 0.2, 0.2) # 샤프트 두께, 헤드 지름, 헤드 길이
        arrow.color = ColorRGBA(1.0, 0.0, 1.0, 1.0) # Magenta

        # 시작점 (차 중심)
        p_start = Point(0, 0, 0.5)
        
        # 끝점 (조향각 반영)
        # steering 값(-1.0 ~ 1.0)은 대략 30도(0.52rad) 정도라고 가정하고 시각화
        steer_rad = steering * 0.5 # 시각화용 스케일링
        arrow_len = 2.0
        p_end = Point(
            arrow_len * np.cos(steer_rad),
            arrow_len * np.sin(steer_rad),
            0.5
        )
        
        arrow.points = [p_start, p_end]
        self.steering_arrow_pub.publish(arrow)

    def publish_target_marker(self, target_point):
        marker = Marker()
        marker.header.frame_id = "fsds/map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = target_point
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(0.5, 0.5, 0.5)
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0) # Green
        self.target_pub.publish(marker)

    def calculate_path_independent(self):
        # 1. 차량 상태 및 콘 필터링
        cx, cy = self.state[0], self.state[1]
        cyaw = np.radians(self.state[2])

        def filter_rear_cones(cones, margin=-2.0):
            filtered = []
            for c in cones:
                dx = c.location.x - cx
                dy = c.location.y - cy
                local_x = dx * np.cos(-cyaw) - dy * np.sin(-cyaw)
                if local_x > margin: filtered.append(c)
            return filtered

        valid_blue = filter_rear_cones(self.blue_cones)
        valid_yellow = filter_rear_cones(self.yellow_cones)

        bp = [(c.location.x, c.location.y) for c in valid_blue]
        yp = [(c.location.x, c.location.y) for c in valid_yellow]

        if len(bp) < 2 or len(yp) < 2:
            self.mid_points = []
            return

        # ----------------------------------------------------------------------
        # [STEP 1] Cone Matching (Pairing)
        # 최적화를 위해 Blue와 Yellow가 1:1로 매칭된 데이터셋을 만들어야 함
        # ----------------------------------------------------------------------
        
        # Primary(많은 쪽) / Secondary(적은 쪽) 설정
        is_primary_blue = len(bp) > len(yp)
        if is_primary_blue:
            primary, secondary = np.array(bp), np.array(yp)
        else:
            primary, secondary = np.array(yp), np.array(bp)

        # Primary 정렬 (Greedy Nearest Neighbor)
        car_pos = np.array([cx, cy])
        primary_sorted = []
        curr = car_pos
        p_copy = primary.tolist()

        while p_copy:
            dists = cdist([curr], p_copy)[0]
            idx = np.argmin(dists)
            if len(primary_sorted) > 0 and dists[idx] > MAX_CONE_CONNECT_DIST: break
            if len(primary_sorted) == 0 and dists[idx] > 15.0: break # 첫 점 너무 멀면 패스

            closest = p_copy.pop(idx)
            primary_sorted.append(closest)
            curr = closest
        
        if len(primary_sorted) < 2: return

        # Matching & Extract Pairs for Optimization
        matched_blue = []
        matched_yellow = []
        
        TRACK_WIDTH_MAX = 10.0 # 매칭 허용 거리

        if len(secondary) > 0:
            for p_pt in primary_sorted:
                dists = cdist([p_pt], secondary)[0]
                min_idx = np.argmin(dists)
                
                if dists[min_idx] < TRACK_WIDTH_MAX:
                    s_pt = secondary[min_idx]
                    
                    # Primary가 Blue였으면 p_pt가 Blue
                    if is_primary_blue:
                        matched_blue.append(p_pt)
                        matched_yellow.append(s_pt)
                    else:
                        matched_blue.append(s_pt)
                        matched_yellow.append(p_pt)

        # 매칭된 쌍이 너무 적으면 경로 생성 불가
        if len(matched_blue) < 3:
            self.mid_points = []
            return

        # Numpy 변환
        matched_blue = np.array(matched_blue)
        matched_yellow = np.array(matched_yellow)

        # ----------------------------------------------------------------------
        # [STEP 2] Centerline Spline Generation
        # 최적화의 기준이 될 중심선 곡률을 구하기 위해 먼저 스플라인 생성
        # ----------------------------------------------------------------------
        try:
            # 단순 중점
            center_points = (matched_blue + matched_yellow) / 2
            
            # 중복 점 제거
            unique_center = [center_points[0]]
            unique_indices = [0]
            for i in range(1, len(center_points)):
                if np.linalg.norm(center_points[i] - unique_center[-1]) > 0.5:
                    unique_center.append(center_points[i])
                    unique_indices.append(i)
            
            unique_center = np.array(unique_center)
            if len(unique_center) < 3: return

            # 필터링된 인덱스에 맞는 Blue/Yellow 배열 재구성 (차원 맞추기)
            matched_blue = matched_blue[unique_indices]
            matched_yellow = matched_yellow[unique_indices]

            # 스플라인 적합 (TCK 생성)
            k_val = min(3, len(unique_center) - 1)
            tck, u = splprep(unique_center.T, k=k_val, s=PATH_SMOOTHING_FACTOR)
            
            # ------------------------------------------------------------------
            # [STEP 3] Racing Line Optimization
            # ------------------------------------------------------------------
            final_path_points = None

            if OPTIMIZE_RACING_LINE:
                # 최적화 수행
                optimized_pts = self.optimize_racing_line(matched_blue, matched_yellow, tck, u)
                
                if optimized_pts is not None:
                    # 최적화 성공 시 해당 점들을 다시 스플라인으로 부드럽게 연결
                    try:
                        tck_opt, _ = splprep(optimized_pts.T, k=k_val, s=0.1) # 최적화된 점은 s를 작게
                        u_new = np.linspace(0, 1, len(unique_center) * PATH_DENSITY)
                        x_new, y_new = splev(u_new, tck_opt)
                        final_path_points = np.column_stack((x_new, y_new))
                    except:
                        # 최적화 후 스플라인 실패 시 그냥 최적화된 점 사용 (보간 없이)
                        final_path_points = optimized_pts
            
            # 최적화를 안 하거나 실패했으면 기본 중심선 사용
            if final_path_points is None:
                u_new = np.linspace(0, 1, len(unique_center) * PATH_DENSITY)
                x_new, y_new = splev(u_new, tck)
                final_path_points = np.column_stack((x_new, y_new))

            # 결과 저장
            self.mid_points = []
            for pt in final_path_points:
                p = Point()
                p.x, p.y = pt[0], pt[1]
                self.mid_points.append(p)

            # 시각화 (Boundary)
            self.publish_boundary_line(matched_blue, "blue")
            self.publish_boundary_line(matched_yellow, "yellow")

        except Exception as e:
            rospy.logerr(f"Path Generation Error: {e}")
            self.mid_points = []

    def get_curvature(self, tck, u):
        """ 스플라인 TCK 정보로 곡률 계산 (최적화 비용 함수용) """
        try:
            deriv1 = splev(u, tck, der=1) # 속도 벡터
            deriv2 = splev(u, tck, der=2) # 가속도 벡터
            x_dot, y_dot = deriv1[0], deriv1[1]
            x_ddot, y_ddot = deriv2[0], deriv2[1]
            
            numerator = np.abs(x_dot * y_ddot - y_dot * x_ddot)
            denominator = (x_dot**2 + y_dot**2)**(3/2)
            
            # 0 나누기 방지
            curvature = np.zeros_like(numerator)
            mask = denominator > 1e-6
            curvature[mask] = numerator[mask] / denominator[mask]
            return curvature
        except:
            return np.zeros_like(u)

    def optimize_racing_line(self, blue_arr, yellow_arr, center_tck, u_vals):
        """
        [NEW] Scipy Optimize를 사용한 레이싱 라인 생성
        - 입력: 매칭된 파란 콘 배열, 노란 콘 배열, 중심선 스플라인 정보
        - 출력: 최적화된 경로 점들 (List of Points)
        """
        try:
            B = blue_arr
            Y = yellow_arr
            n_points = len(B)

            # 1. 중심선의 곡률 계산 (직선/곡선 판단용)
            curvatures = self.get_curvature(center_tck, u_vals)
            
            # 2. 가중치 계산 (곡률이 클수록 center_weight 감소 -> 최단 거리 우선)
            # alpha가 클수록 미세한 커브도 곡선으로 인식
            center_weights = 1.0 / (1.0 + RACING_LINE_ALPHA * curvatures)

            # 3. 비용 함수 (Cost Function)
            def adaptive_cost(w):
                # w는 0(Blue) ~ 1(Yellow) 사이의 비율
                w_expanded = w[:, np.newaxis]
                
                # 현재 w에 따른 경로 점 P
                P = (1 - w_expanded) * B + w_expanded * Y
                
                # A. 경로 길이 비용 (최단 거리 추구 -> Out-In-Out 유도)
                diff = P[1:] - P[:-1]
                sq_dist = np.sum(diff**2)
                
                # B. 중앙 유지 비용 (직선 구간 안정성)
                # (w - 0.5)^2 : 중앙에서 벗어날수록 페널티
                center_deviation = np.sum(center_weights * (w - 0.5)**2)
                
                return sq_dist + RACING_LINE_BETA * center_deviation

            # 4. 최적화 수행
            initial_w = np.full(n_points, 0.5) # 초기값은 중앙
            
            # 경계값 설정 (콘에 박지 않도록 안전 마진 확보)
            bounds = [(SAFETY_MARGIN, 1.0 - SAFETY_MARGIN) for _ in range(n_points)]
            
            # 실시간성을 위해 허용 오차(tol)를 조금 크게 잡거나 반복 횟수를 제한할 수 있음
            res = minimize(adaptive_cost, initial_w, method='L-BFGS-B', bounds=bounds, tol=1e-2)

            if res.success:
                opt_w = res.x
                w_expanded = opt_w[:, np.newaxis]
                # 최적화된 좌표 계산
                optimal_path = (1 - w_expanded) * B + w_expanded * Y
                return optimal_path
            else:
                return None

        except Exception as e:
            rospy.logwarn(f"Optimization Failed: {e}")
            return None
  
    def calculate_path_curvature(self, p1, p2, p3):
        x1, y1 = p1.x, p1.y
        x2, y2 = p2.x, p2.y
        x3, y3 = p3.x, p3.y
        a = np.hypot(x1 - x2, y1 - y2)
        b = np.hypot(x2 - x3, y2 - y3)
        c = np.hypot(x3 - x1, y3 - y1)
        if (a * b * c) == 0: return 0.0
        s = (a + b + c) / 2.0
        area = np.sqrt(abs(s * (s - a) * (s - b) * (s - c)))
        return min((4 * area) / (a * b * c), 1.0)
    
    def calculate_curvature_speed(self, current_idx, current_v):
        if not self.mid_points or len(self.mid_points) < 3:
            return MAX_SPEED, 0.0

        # 1. 예측 구간 설정
        predict_dist = 10.0 
        k_list = []
        
        accumulated_dist = 0.0
        start_scan = current_idx
        end_scan = len(self.mid_points) - 2
        
        for i in range(start_scan, end_scan):
            p1 = self.mid_points[i]
            p2 = self.mid_points[i+1]
            p3 = self.mid_points[i+2]
            
            d = np.hypot(p2.x - p1.x, p2.y - p1.y)
            accumulated_dist += d
            if accumulated_dist > predict_dist:
                break

            k = self.calculate_path_curvature(p1, p2, p3)
            k_list.append(k)

        if not k_list:
            return MAX_SPEED, 0.0

        # 2. [Robust Max] 상위 3개 평균 (노이즈 제거)
        k_list.sort(reverse=True)
        top_n = 3
        count = min(len(k_list), top_n)
        robust_k = sum(k_list[:count]) / count

        # ===============================================================
        # 3. [핵심 수정] 곡률 범위(0.05 ~ 0.1) 에 따른 비례 속도 제어
        # ===============================================================
        
        # 튜닝 값 설정 (사용자 데이터 기반)
        V_MAX = MAX_SPEED      # 직선 최고 속도
        V_MIN = MIN_SPEED      # 코너 최저 속도 (3.0은 너무 느릴 수 있어 4.0 추천)

        target_v = V_MAX

        if robust_k <= K_STRAIGHT:
            # 거의 직선 -> 풀 악셀
            target_v = V_MAX
            
        elif robust_k >= K_CORNER_MAX:
            # 급커브(라고 판단되는 0.1 이상) -> 최저 속도 유지
            target_v = V_MIN
            
        else:
            # 그 사이 구간 (0.05 ~ 0.1) -> 선형적으로 속도 줄임 (Mapping)
            # ratio: 0(직선) ~ 1(최대코너) 사이의 비율
            ratio = (robust_k - K_STRAIGHT) / (K_CORNER_MAX - K_STRAIGHT)
            
            # 비율만큼 속도 차감
            target_v = V_MAX - ratio * (V_MAX - V_MIN)

        return target_v, robust_k
    
    def pure_pursuit(self):
        # [최적화] calculate_path_independent는 track_callback에서만 호출하고 여기서는 뺍니다.
        self.calculate_path_independent() 

        x, y, theta_deg, v = self.state
        yaw_rad = np.radians(theta_deg)
        
        if not self.mid_points:
            return 0.0, 0.0, 0.0 # 안전하게 0 리턴

        # --- (1) Lookahead 거리 튜닝 (안정성 확보) ---
        # 시뮬레이터에서는 노이즈 때문에 최소 거리를 넉넉하게 잡는 게 유리합니다.
        # 기존 2.5 -> 4.0~5.0 정도로 늘려보세요. S자 탈출의 핵심입니다.
        current_lookahead = np.clip(LOOKAHEAD_MIN + (LOOKAHEAD_GAIN * v), LOOKAHEAD_MIN, LOOKAHEAD_MAX)
        # 3 ~ 10
        # 최대 속도 10m/s일 때 4 + 0.1*10 = 5.0m
        # 실제로는 3 ~ 4 m 

        # --- (2) 타겟 포인트 찾기 ---
        # 가장 가까운 점 찾기
        dists = [np.hypot(p.x - x, p.y - y) for p in self.mid_points]
        min_dist_idx = np.argmin(dists)

        # Lookahead 거리보다 먼 첫 번째 점 찾기
        target_idx = min_dist_idx
        found = False
        for i in range(min_dist_idx, len(self.mid_points)):
            # 누적 거리가 아니라 단순 유클리드 거리로 비교 (연산 속도)
            if dists[i] > current_lookahead:
                target_idx = i
                found = True
                break
        
        # 만약 트랙 끝부분이라 Lookahead보다 먼 점이 없으면 마지막 점을 타겟으로 (?)
        if not found:
            target_idx = len(self.mid_points) - 1

        target_point = self.mid_points[target_idx]

        # --- (3) 조향각 계산 (Pure Pursuit) ---
        dx = target_point.x - x
        dy = target_point.y - y
        local_x = dx * np.cos(-yaw_rad) - dy * np.sin(-yaw_rad)
        local_y = dx * np.sin(-yaw_rad) + dy * np.cos(-yaw_rad)
        
        # 후방에 있는 점을 타겟으로 잡는 경우(U턴 등) 방지
        if local_x < 0:
             local_x = 0.1 

        dist_squared = local_x**2 + local_y**2
        
        # Pure Pursuit 공식
        raw_steering = np.arctan(2 * WHEELBASE * local_y / dist_squared)
        
        # [Fix] Gain 적용하되, 물리적 한계(-1.0 ~ 1.0) 내에서 클리핑
        # STEERING_GAIN을 1.2 정도로 낮추는 것을 추천합니다.
        desired_steering = np.clip(raw_steering * STEERING_GAIN * MIRROR_STEERING, -1.0, 1.0)

        # [Fix] Steering Smoothing (Low Pass Filter)
        # 갑작스러운 핸들 털림 방지. alpha가 작을수록 부드럽지만 반응이 느려짐 (0.1 ~ 0.3 추천)
        # steer_alpha = 0.3
        # steering = (self.prev_steering * (1 - steer_alpha)) + (desired_steering * steer_alpha)
        steering = desired_steering
        self.prev_steering = steering

        # --- (4) 속도 제어 (Curvature + Smoothing) ---
        
        # 곡률 기반 목표 속도 계산
        curve_v, max_k = self.calculate_curvature_speed(min_dist_idx, v)
        
        # 조향각이 크면 감속 (즉각적인 반응)
        if abs(steering) > 0.3: # 0.3 rad 이상 꺾으면
            curve_v = min(curve_v, MIN_SPEED + 2.0)

        # [Fix] 목표 속도 스무딩 (Target Velocity Smoothing)
        # 목표 속도가 10 -> 3 -> 10 으로 급변하는 것을 방지
        v_alpha = V_ALPHA
        target_v = (self.prev_target_v * (1 - v_alpha)) + (curve_v * v_alpha)
        self.prev_target_v = target_v
        
        # 최소 속도 보장
        target_v = max(target_v, MIN_SPEED)

        # --- (5) 가감속 제어 (PID + Deadzone) ---
        speed_error = target_v - v
        throttle = 0.0
        brake = 0.0

        # [Fix] Deadzone (히스테리시스) 추가
        # 속도 오차가 ±0.5 m/s 이내면 엑셀/브레이크 조작 안 함 (울컥거림 방지 핵심)
        if abs(speed_error) < 0.5:
            throttle = 0.0
            brake = 0.0
        elif speed_error > 0:
            # 가속
            throttle = np.clip(K_ACCEL * speed_error, 0.0, MAX_ACCEL)
            # 급격한 엑셀링 방지 (Smoothing)
            throttle = (self.prev_throttle * 0.7) + (throttle * 0.3)
        else:
            # 감속
            brake = np.clip(K_BRAKE * abs(speed_error), 0.0, MAX_BRAKE)
            # 급격한 브레이킹 방지 (Smoothing) - 브레이크는 좀 더 빨리 반응해도 됨
            brake = (self.prev_brake * 0.5) + (brake * 0.5)

        self.prev_throttle = throttle
        self.prev_brake = brake

        # [VISUALIZATION]
        self.publish_target_marker(target_point)
        self.publish_debug_dashboard(target_v, throttle, brake, steering, max_k)

        return throttle, steering, brake
        
    def run(self):
        x, y, theta, v = self.state
        path = self.mid_points

        # --------------------------------------------------------------------------

        throttle, steering, brake = self.pure_pursuit()

        # --------------------------------------------------------------------------

        cmd = ControlCommand()
        cmd.header.stamp = rospy.Time.now()
        cmd.throttle = throttle
        cmd.steering = steering
        cmd.brake = brake
        self.cmd_pub.publish(cmd)

        self.publish_midpoints()
        self.cones_pub.publish(self.cone_markers)

def main():
    rospy.init_node("control_node")
    node = ControlNode()

    rate = rospy.Rate(30) # 30 Hz
    while not rospy.is_shutdown():
        node.run()
        rate.sleep()

if __name__ == "__main__":
    main()
