#!/usr/bin/env python3

import rospy
import numpy as np
import math
import sys

from database import Database
from ackermann_msgs.msg import AckermannDrive
from parameter_list import Param
from debug import BrainLogger
from visualization_msgs.msg import Marker

param = Param()

class ParkingWiggleController:
    def __init__(self):
        self.state = "IDLE"  # 상태: IDLE, CHECK, FORWARD, BACKWARD, FINISHED
        self.start_x = 0.0
        self.start_y = 0.0
        self.wiggle_count = 0
        
        # --- 튜닝 파라미터 ---
        self.TARGET_YAW = 0.0          # 목표 각도 (라디안)
        self.TARGET_Y = 8.00           # 목표 Y 좌표
        self.YAW_TOLERANCE = math.radians(5.0) # 허용 오차 (5도) - 완화됨
        self.LAT_TOLERANCE = 0.15      # 허용 오차 (15cm)
        self.MOVE_DIST = 0.5           # 한 번 움직일 거리 (m)
        self.SPEED = 3.0               # 이동 속도
        self.MAX_STEER = math.radians(30.0) # 최대 조향각
        self.MAX_WIGGLE_COUNT = 10     # 최대 시도 횟수 (무한루프 방지)
        
    def reset(self):
        self.state = "IDLE"
        self.wiggle_count = 0

    def update(self, current_x, current_y, current_yaw):
        # 1. 오차 계산
        # 각도 오차 (-PI ~ PI 정규화)
        yaw_error = current_yaw - self.TARGET_YAW
        while yaw_error > math.pi: yaw_error -= 2 * math.pi
        while yaw_error < -math.pi: yaw_error += 2 * math.pi
        
        # 측면 오차 (현재 Y - 목표 Y)
        lat_error = current_y - self.TARGET_Y
        
        # 2. 상태 머신
        if self.state == "IDLE":
            self.state = "CHECK"
            return 0, 0, False

        elif self.state == "CHECK":
            # [추가] yaw가 -10~10도 사이이고 stop point 근처면 즉시 종료
            yaw_deg = math.degrees(current_yaw)
            stop_point_x, stop_point_y = 9.9, 8.00
            dist_to_stop = math.hypot(current_x - stop_point_x, current_y - stop_point_y)
            
            if -10.0 <= yaw_deg <= 10.0 and dist_to_stop < 0.5:
                print(f"✅ Already near aligned! (Yaw: {yaw_deg:.1f}deg, Dist: {dist_to_stop:.2f}m)")
                self.state = "FINISHED"
                return 0, 0, True
            
            # 정렬 완료 확인 (각도 AND 위치)
            is_yaw_ok = abs(yaw_error) < self.YAW_TOLERANCE
            is_lat_ok = abs(lat_error) < self.LAT_TOLERANCE
            
            # 최대 횟수 초과 시 강제 종료
            if (is_yaw_ok and is_lat_ok) or self.wiggle_count >= self.MAX_WIGGLE_COUNT:
                if self.wiggle_count >= self.MAX_WIGGLE_COUNT:
                    print("⚠️ Wiggle Timeout! Forcing Exit.")
                self.state = "FINISHED"
                return 0, 0, True
            
            # 정렬 안됨 -> 움직일 방향 결정
            self.start_x = current_x
            self.start_y = current_y
            
            # 짝수: 전진, 홀수: 후진
            if self.wiggle_count % 2 == 0:
                self.state = "FORWARD"
            else:
                self.state = "BACKWARD"
            
            return 0, 0, False

        elif self.state == "FORWARD":
            dist = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
            if dist >= self.MOVE_DIST:
                self.wiggle_count += 1
                self.state = "CHECK"
                return 0, 0, False
            
            # 조향 로직 (Mixed Control)
            # Yaw Error > 0 (Left) -> Steer Right (-)
            # Lat Error > 0 (High) -> Steer Right (-) to go Down
            k_yaw = 1.5
            k_lat = 1.0
            
            steer = - (k_yaw * yaw_error + k_lat * lat_error)
            steer = max(min(steer, self.MAX_STEER), -self.MAX_STEER)
            
            return self.SPEED, steer, False

        elif self.state == "BACKWARD":
            dist = math.sqrt((current_x - self.start_x)**2 + (current_y - self.start_y)**2)
            if dist >= self.MOVE_DIST:
                self.wiggle_count += 1
                self.state = "CHECK"
                return 0, 0, False
            
            # 조향 로직 (Mixed Control)
            # Yaw Error > 0 (Left) -> Steer Left (+) to turn Rear Left (Front Right) -> Yaw decreases
            # Lat Error > 0 (High) -> Steer Right (-) to turn Rear Right (Down) -> Y decreases
            k_yaw = 1.5
            k_lat = 1.0
            
            steer = (k_yaw * yaw_error) - (k_lat * lat_error)
            steer = max(min(steer, self.MAX_STEER), -self.MAX_STEER)
            
            return -self.SPEED, steer, False

        elif self.state == "FINISHED":
            return 0, 0, True
            
        return 0, 0, False

class Brain():
    def __init__(self, db=None, map_number=int):
        if isinstance(db, Database):
            self.db = db
        else:
            self.db = Database(lidar=True)
            
        self.debug_pub = rospy.Publisher('/debug_marker', Marker, queue_size=1)
        
        self.target_speed = 0.0
        self.MAX_STEER = math.radians(30.0) # 최대 조향각 (Brain 클래스용)
        self.s_phase = 0 
        
        # ==========================================
        # 🧠 Parking Logic Variables (이식됨)
        # ==========================================
        self.mission_state = 0      # 각 미션 내부 단계
        self.state_timer = 0        # 시간 측정용
        
        # Recorded Paths for Reverse
        self.p1_record_path = [] 
        
        # 미션 완료 플래그
        self.completed = {
            'P1': False, # 전면 주차
            'P3': False  # 평행 주차
        }

        # 📍 P1 전면 주차 웨이포인트
        self.p1_path = [
            (3.92, 4.49),  # Start
            (3.40, 4.4),
            (2.91, 4.32),
            (2.65, 4.28),
            (2.54, 3.94),
            (2.53, 3.41),
            (2.52, 2.40)   # Final (deeper entry - changed from 2.60)
        ]

        # 📍 P3 평행 주차 웨이포인트 (Original)
        raw_p3_path = [
            (8.32, 9.00),
            (8.69, 8.93),
            (8.85, 8.87),
            (8.96, 8.71),
            (9.06, 8.23),  # 여기서부터 -0.3 적용됨 (원래 8.53)
            (9.12, 8.14),  # (원래 8.44)
            (9.23, 8.01),  # (원래 8.31)
            (9.48, 7.93),  # (원래 8.13)
            (9.48, 7.83),  # Stop Point (Index 8) -0.3 적용됨 (원래 8.03)
            (9.68, 7.93),  # (원래 8.13) 
            (9.93, 8.01),  # (원래 8.31)
            (10.04, 8.14), # (원래 8.44)
            (10.10, 8.23), # 여기까지 -0.3 적용됨 (원래 8.53)
            (10.20, 8.71),
            (10.31, 8.87),
            (10.47, 8.93),
            (10.84, 9.00)
        ]
        
        # [Added] 경로 세분화 (0.05m 간격)
        self.p3_path = self.interpolate_path(raw_p3_path, 0.05)
        
        self.wiggle_controller = ParkingWiggleController()

    def interpolate_path(self, path, interval):
        new_path = []
        for i in range(len(path) - 1):
            p1 = np.array(path[i])
            p2 = np.array(path[i+1])
            dist = np.linalg.norm(p2 - p1)
            num_points = int(dist / interval)
            for j in range(num_points):
                alpha = j / num_points
                p = (1 - alpha) * p1 + alpha * p2
                new_path.append(tuple(p))
        new_path.append(path[-1])
        return new_path

    def publish_target_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3; marker.scale.y = 0.3; marker.scale.z = 0.3
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        self.debug_pub.publish(marker)

    def main(self):
        # ======================= 데이터 수신 =======================
        lidar_data = self.db.lidar_data
        pose_data = self.db.pose_data # [x, y, yaw_deg]
        global_path = self.db.global_path
        
        if len(global_path) == 0:
            print("Waiting for global path...")
            return 0, 0

        curr_mission = self.db.current_mission
        traffic_light = self.db.traffic_light
        
        x, y = pose_data[0], pose_data[1]
        yaw_deg = pose_data[2]
        
        # =========================================================
        # ️ 구간(Zone) 판단
        # =========================================================
        
        is_s_zone = (5.0 < x < 8.5) and (y < 6.0)
        # 동적 장애물 구간 (x=5.8 부근 집중 관리)
        is_dynamic_obs = (5.5 < x < 7.5) and (4.5 < y < 5.5)
        
        # =========================================================
        # 🚗 주행 로직
        # =========================================================
        
        angle = 0.0
        speed = 0.0

        # [Mission 1] 주차 (Location Based Force Trigger)
        # curr_mission 값과 상관없이 위치가 주차 구역이면 실행
        
        # P1 시작점과의 거리 계산 (P1 Start: 3.92, 4.49)
        dist_p1 = math.hypot(3.92 - x, 4.49 - y)
        # P3 시작점과의 거리 계산 (P3 Start: 8.12, 9.00)
        dist_p3 = math.hypot(8.12 - x, 9.00 - y)
        
        # 1. P1 구역 근처이고 아직 완료 안 했으면 -> P1 실행
        if dist_p1 < 3.0 and not self.completed['P1']:
            return self.parking_logic_1_front(pose_data, lidar_data)
        
        # 2. P3 구역 근처이고 아직 완료 안 했으면 -> P3 실행
        # [Modified] P3 진입 조건 수정: 거리 기반(1.5m)은 Stop Point(9.48) 도달 전 끊김
        # 따라서 x > 7.5 및 y > 7.5 영역으로 넓게 설정하여 Stop Point까지 로직 유지
        elif x > 7.5 and y > 7.5 and not self.completed['P3']:
            return self.parking_logic_3_parallel(pose_data, lidar_data)

        # [Mission 2] 신호등
        if curr_mission == 2 and not (is_s_zone or is_dynamic_obs):
            if traffic_light == "RED" or traffic_light == "YELLOW":
                return 0, 0

        # ---------------------------------------------------------
        # [Zone 1 & 2] S자 코스 + 동적 장애물 (통합 구역)
        # ---------------------------------------------------------
        # 조건: x가 점1 근처거나 주행 중이고, y가 점2보다 작을 때
        if (x > 6.0 and y < 4.93) and not (x < 3.0):
            # 간단히 x 좌표가 큰 구역을 S자 및 동적 장애물 구역으로 간주
            if y < 4.5:  # 점1~점2 가는 길 (S자)
                return self.run_s_course(lidar_data, x, y, yaw_deg)
            else:  # 점2~점3 (동적 장애물)
                return self.run_dynamic_obs(lidar_data, pose_data, global_path)
        
        # ---------------------------------------------------------
        # [Zone 3] 일반 주행 (과감한 회전)
        # ---------------------------------------------------------
        angle, speed = self.pure_pursuit_adaptive(pose_data, global_path, min_ld=0.5, max_ld=1.2)


        # =========================================================
        # 🛡️ 능동적 충돌 회피 (동적 장애물 구간은 제외)
        # =========================================================
        skip_safety = is_dynamic_obs
        
        if not skip_safety:
            safe_angle, safe_speed = self.active_safety_guard(lidar_data, angle, speed)
            self.target_speed = safe_speed
            return safe_angle, safe_speed
        
        self.target_speed = speed
        return angle, speed

    # ============================================================
    # 🅿️ [Integrated] Parking Logic Methods
    # ============================================================

    # 🅿️ [P1] 전면 주차
    def parking_logic_1_front(self, pose, lidar_data):
        x, y, yaw = pose  # Brain passes [x, y, yaw_deg]
        
        # [Phase 0] Path Following (Strict Pure Pursuit)
        if self.mission_state == 0:
            # 경로 기록
            if not hasattr(self, 'p1_record_path'):
                self.p1_record_path = []
            
            if len(self.p1_record_path) == 0 or math.hypot(self.p1_record_path[-1][0] - x, self.p1_record_path[-1][1] - y) > 0.1:
                self.p1_record_path.append((x, y))

            # Check arrival at final point
            final_tx, final_ty = self.p1_path[-1]
            dist_x = abs(final_tx - x)
            dist_y = abs(final_ty - y)
            
            # [Modified] y값이 목표 근처(0.6m 이내)면 멈춤
            if dist_y < 0.6: 
                print(f"📌 [P1] Reached Final Position (Y-Trigger): ({final_tx:.2f}, {final_ty:.2f})")
                self.mission_state = 1
                self.state_timer = rospy.Time.now().to_sec()
                print("🅿️ P1 Final Position! Waiting 3s...")
                return 0, 0

            # Pure Pursuit Calculation
            path_np = np.array(self.p1_path)
            dists = np.hypot(path_np[:,0]-x, path_np[:,1]-y)
            closest_idx = np.argmin(dists)
            
            lookahead = 0.5
            target_idx = closest_idx
            for i in range(closest_idx, len(self.p1_path)):
                if dists[i] > lookahead:
                    target_idx = i
                    break
            
            if target_idx >= len(self.p1_path): target_idx = len(self.p1_path) - 1
            tx, ty = self.p1_path[target_idx]
            self.publish_target_marker(tx, ty) # Debug Marker 추가

            alpha = math.atan2(ty - y, tx - x) - np.radians(yaw)
            while alpha > math.pi: alpha -= 2 * math.pi
            while alpha < -math.pi: alpha += 2 * math.pi
            
            # [Phase 0 - Final Approach Logic] Heading Hold
            if y < 3.5:
                target_yaw = -90.0
                yaw_diff = self.normalize_deg(target_yaw - yaw)
                angle = np.clip(yaw_diff * 0.05, -0.55, 0.55)
                
                print(f"🛡️ [Heading Hold] Y:{y:.2f} < 3.5 | Yaw:{yaw:.1f} -> Tgt:-90 | Steer:{angle:.2f}")
                
                if dist_y < 0.2:
                    print(f"📌 [P1] Reached Final Position (Y-Trigger): ({final_tx:.2f}, {final_ty:.2f})")
                    self.mission_state = 1
                    self.state_timer = rospy.Time.now().to_sec()
                    print("🅿️ P1 Final Position! Waiting 3s...")
                    return 0, 0
                
                return angle, 1.5

            delta = math.atan2(2 * 0.33 * math.sin(alpha) / lookahead, 1.0)
            angle = float(np.clip(delta, -0.55, 0.55))
            
            dist_to_final = math.hypot(dist_x, dist_y)
            self.print_status("P1", "Path Following", f"Dist:{dist_to_final:.2f}", x, y, yaw)
            return angle, 1.5

        # [Phase 1] 3초 정지
        elif self.mission_state == 1:
            if rospy.Time.now().to_sec() - self.state_timer > 3.1:
                self.mission_state = 2
                print("✅ P1 Complete. Exiting (Reverse Path Tracking)...")
            return 0, 0

        # [Phase 2] 후진으로 탈출
        elif self.mission_state == 2:
            path_len = len(self.p1_record_path)
            if path_len < 2:
                self.mission_state = 3
                return 0, 0
            
            min_dist = float('inf')
            closest_idx = 0
            for i in range(path_len):
                d = math.hypot(self.p1_record_path[i][0] - x, self.p1_record_path[i][1] - y)
                if d < min_dist:
                    min_dist = d
                    closest_idx = i
            
            lookahead = 0.8
            target_idx = closest_idx
            for i in range(closest_idx, -1, -1):
                d = math.hypot(self.p1_record_path[i][0] - x, self.p1_record_path[i][1] - y)
                if d > lookahead:
                    target_idx = i
                    break
            
            # 탈출 조건 확인
            sx, sy = 3.92, 4.49
            dist_to_start = math.hypot(sx - x, sy - y)
            
            rear_dist = float('inf')
            if lidar_data is not None:
                ranges = None
                if hasattr(lidar_data, 'ranges'):
                    ranges = lidar_data.ranges
                elif isinstance(lidar_data, (list, tuple, np.ndarray)):
                    ranges = lidar_data
                
                if ranges is not None and len(ranges) > 0:
                    mid_idx = len(ranges) // 2
                    start_idx = max(0, mid_idx - 10)
                    end_idx = min(len(ranges), mid_idx + 10)
                    rear_ranges = ranges[start_idx:end_idx]
                    if len(rear_ranges) > 0:
                        rear_dist = np.min(rear_ranges)

            if (target_idx == 0 and min_dist < 0.5) or dist_to_start < 0.5 or rear_dist < 0.5:
                if rear_dist < 0.5:
                    print("⚠️ Rear Wall Detected! Starting Recovery Turn...")
                else:
                    print("✅ Reached Start Point. Starting Recovery Turn...")
                
                self.mission_state = 3
                self.state_timer = rospy.Time.now().to_sec()
                return 0, 0

            tx, ty = self.p1_record_path[target_idx]
            
            dx = tx - x
            dy = ty - y
            yaw_rad = np.radians(yaw)
            local_x = math.cos(-yaw_rad) * dx - math.sin(-yaw_rad) * dy
            local_y = math.sin(-yaw_rad) * dx + math.cos(-yaw_rad) * dy
            
            ld2 = dx**2 + dy**2
            steer = math.atan(2 * 0.33 * local_y / ld2)
            steer = np.clip(steer, -0.55, 0.55)
            
            self.print_status("P1", "Reversing", f"TgtIdx:{target_idx} Rear:{rear_dist:.2f}", x, y, yaw)
            return steer, -1.5

        # [Phase 3] 0.3초간 우회전 전진 (Recovery Turn) - 전역 경로 복귀
        elif self.mission_state == 3:
            if rospy.Time.now().to_sec() - self.state_timer > 0.3:
                self.mission_state = 4
                return 0, 0
            
            # 단순히 앞으로 직진
            self.print_status("P1", "Recovery", "Forward", x, y, yaw)
            return 0.0, 2.0  # 직진

        # [Phase 4] 종료
        elif self.mission_state == 4:
            print("✅ P1 Mission Complete. Resuming Normal Drive.")
            self.completed['P1'] = True
            self.mission_state = 0
            return 0, 0
            
        return 0, 0

    # 🅿️ [P3] 평행 주차
    def parking_logic_3_parallel(self, pose, lidar_data):
        x, y, yaw = pose
        
        # [Phase 0] Path Following (Forward Entry)
        if self.mission_state == 0:
            # Stop Point: (9.48, 7.95)
            stop_tx, stop_ty = 9.48, 7.95
            
            # [Modified] 정차 조건 변경: 9.2 < x < 9.8 이거나 yaw가 -10~10 사이
            # 단, yaw 조건은 시작 지점(x < 8.8)에서의 오작동 방지를 위해 x > 8.8 추가
            is_in_x_range = (x>=9.4)
            is_aligned = (-10.0 <= yaw <= 10.0) and (x > 8.8)
            
            if is_in_x_range or is_aligned:
                
                
                self.mission_state = 5  # Phase 5 (Twist Forward)
                self.state_timer = rospy.Time.now().to_sec()
                print(f"🛑 [P3] Stop Condition Met! x:{x:.2f}, yaw:{yaw:.2f}")
                print("🔄 Performing Final Alignment (Twist)...")
                return 0, 0

            # Pure Pursuit (Targeting up to Index 8)
            # [Modified] Use densified path, find index of stop point
            # Stop point is roughly at index where x ~ 9.48
            # For simplicity, just target a point ahead
            
            # Find closest point on densified path
            path_np = np.array(self.p3_path)
            dists = np.hypot(path_np[:,0]-x, path_np[:,1]-y)
            closest_idx = np.argmin(dists)
            
            lookahead = 0.6
            target_idx = closest_idx
            for i in range(closest_idx, len(self.p3_path)):
                if dists[i] > lookahead:
                    target_idx = i
                    break
            
            # Limit target index to stop point (9.48, 8.03)
            # We can find the index of the stop point in the densified path
            # Or just check if target point is beyond stop point
            
            tx, ty = self.p3_path[target_idx]
            
            # If target is beyond stop point (x > 9.48), clamp it? 
            # Actually, let's just let it follow the path. The stop condition above handles the stop.
            
            self.publish_target_marker(tx, ty) # Debug Marker

            alpha = math.atan2(ty - y, tx - x) - np.radians(yaw)
            while alpha > math.pi: alpha -= 2 * math.pi
            while alpha < -math.pi: alpha += 2 * math.pi
            
            delta = math.atan2(2 * 0.33 * math.sin(alpha) / lookahead, 1.0)
            angle = float(np.clip(delta, -0.55, 0.55))
            
            dist_to_stop = math.hypot(stop_tx - x, stop_ty - y)
            self.print_status("P3", "Entry", f"Dist:{dist_to_stop:.2f}", x, y, yaw)
            return angle, 1.5

        # [Phase 5] Twist Forward (+30 deg steer, forward)
        elif self.mission_state == 5:
            # [Modified] Yaw가 -10~10도 사이면 Twist 생략
            if -10.0 <= yaw <= 10.0:
                self.mission_state = 2
                self.state_timer = rospy.Time.now().to_sec()
                print("✅ Already aligned! Skipping Twist.")
                return 0, 0

            if rospy.Time.now().to_sec() - self.state_timer > 0.5: # 0.5초 이동
                self.mission_state = 6
                self.state_timer = rospy.Time.now().to_sec()
                return 0, 0
            return self.MAX_STEER, 0.5

        # [Phase 6] Twist Backward (+30 deg steer, backward)
        elif self.mission_state == 6:
            if rospy.Time.now().to_sec() - self.state_timer > 0.5: # 0.5초 이동
                self.mission_state = 2
                self.state_timer = rospy.Time.now().to_sec()
                print("✅ Alignment Complete! Waiting 3s...")
                return 0, 0
            return self.MAX_STEER, -0.5

        # [Phase 1] Wiggle (Shuffling)
        elif self.mission_state == 1:
            speed, steer, finished = self.wiggle_controller.update(x, y, np.radians(yaw))
            
            if finished:
                self.mission_state = 2
                self.state_timer = rospy.Time.now().to_sec() # 대기 타이머 시작
                print("🚀 Wiggle Complete! Waiting 3s before exit...")
                return 0, 0
            else:
                self.print_status("P3", "Wiggle", self.wiggle_controller.state, x, y, yaw)
                return steer, speed

        # [Phase 2] Wait 3s
        elif self.mission_state == 2:
            if rospy.Time.now().to_sec() - self.state_timer > 3.1:
                self.mission_state = 3
                print("🚀 P3 Resuming Forward Path (Exit)...")
            return 0, 0

        # [Phase 3] Continue Path Following (Exit Forward)
        elif self.mission_state == 3:
            # Remaining Path: Index 8 to End
            target_sub_path = self.p3_path
            
            # Check arrival at final point
            final_tx, final_ty = self.p3_path[-1]
            dist_to_final = math.hypot(final_tx - x, final_ty - y)
            
            if dist_to_final < 0.5: # End of P3
                print("✅ P3 Mission Complete.")
                self.completed['P3'] = True
                self.mission_state = 0
                return 0, 0

            # Pure Pursuit on full path
            path_np = np.array(target_sub_path)
            dists = np.hypot(path_np[:,0]-x, path_np[:,1]-y)
            closest_idx = np.argmin(dists)
            
            lookahead = 0.6
            target_idx = closest_idx
            for i in range(closest_idx, len(target_sub_path)):
                if dists[i] > lookahead:
                    target_idx = i
                    break
            if target_idx >= len(target_sub_path): target_idx = len(target_sub_path) - 1
            tx, ty = target_sub_path[target_idx]
            self.publish_target_marker(tx, ty) # Debug Marker
            
            alpha = math.atan2(ty - y, tx - x) - np.radians(yaw)
            while alpha > math.pi: alpha -= 2 * math.pi
            while alpha < -math.pi: alpha += 2 * math.pi
            
            delta = math.atan2(2 * 0.33 * math.sin(alpha) / lookahead, 1.0)
            angle = float(np.clip(delta, -0.55, 0.55))
            
            self.print_status("P3", "Exit", f"Dist:{dist_to_final:.2f}", x, y, yaw)
            return angle, 1.5

        return 0, 0

    # ============================================================
    # 🛠️ Helper Functions
    # ============================================================
    
    def normalize_deg(self, angle):
        while angle > 180: angle -= 360
        while angle < -180: angle += 360
        return angle

    def print_status(self, mission, state, reason, x, y, yaw):
        msg = f"\r[Brain] {mission} | {state} ({reason}) | Pos:({x:.1f},{y:.1f})      "
        sys.stdout.write(msg)
        sys.stdout.flush()

    def get_closest_parking_spot(self, car_x, car_y):
        # This method is less relevant now with hardcoded paths, but kept for compatibility
        closest_spot = None
        min_dist = float('inf')
        # Original vision based logic if needed
        return closest_spot

    def navigate_s_curve(self, lidar):
        if len(lidar) == 0: return 0.0
        center = len(lidar) // 2
        left_avg = np.nanmean(lidar[center+30 : center+90])
        right_avg = np.nanmean(lidar[center-90 : center-30])
        if np.isnan(left_avg): left_avg = 5.0
        if np.isnan(right_avg): right_avg = 5.0
        return np.clip(0.4 * (left_avg - right_avg), -0.5, 0.5)

    def active_safety_guard(self, lidar, cmd_angle, cmd_speed):
        if len(lidar) == 0: return cmd_angle, cmd_speed
        min_dist = np.nanmin(lidar)
        min_idx = np.nanargmin(lidar)
        center = len(lidar) // 2
        
        if min_dist < 0.25:
            if min_idx > center: # 왼쪽 장애물
                return 0.6, -1.5 # 핸들 좌측, 후진 (앞머리 우회전)
            else: # 오른쪽 장애물
                return -0.6, -1.5 # 핸들 우측, 후진 (앞머리 좌회전)
                
        elif min_dist < 0.45:
            if min_idx > center: # 왼쪽 장애물
                return -0.6, 0.6 # 핸들 우측, 전진 (우측 회피)
            else: # 오른쪽 장애물
                return 0.6, 0.6 # 핸들 좌측, 전진 (좌측 회피)
        return cmd_angle, cmd_speed

    # ============================================================
    # ⚙️ 기타 미션 로직
    # ============================================================
    def run_s_course(self, lidar, x, y, yaw):
        # 점1(7.25, 0.37) -> 점2(7.25, 4.93) S자 주행
        # 간단한 라이다 기반 중앙 유지 + 전진
        angle = self.navigate_s_curve(lidar)
        return angle, 8.0  # 빠르게 주행 (increased from 5.0)

    def run_dynamic_obs(self, lidar, pose, path):
        # 장애물 없으면 빠르게
        front_dist = self.get_front_distance(lidar, fov=20)
        if front_dist > 3.0:
            # MAX_SPEED로 주행, pure_pursuit 사용 (lookahead 2.0)
            angle, _ = self.pure_pursuit(pose, path, 20.0)
            return angle, 20.0
        else:
            # 장애물 있으면 잠시 대기 or 서행
            angle, _ = self.pure_pursuit(pose, path, 20.0)
            return angle, 20.0  # 안전제일

    def pure_pursuit(self, pose, path, speed_limit):
        x, y, yaw_deg = pose
        yaw = np.radians(yaw_deg)
        path_np = np.array(path)
        dists = np.hypot(path_np[:,0]-x, path_np[:,1]-y)
        closest_idx = np.argmin(dists)
        lookahead = 2.0  # Lookahead distance
        
        target_idx = closest_idx
        for i in range(closest_idx, len(path)):
            if dists[i] > lookahead:
                target_idx = i
                break
        
        if target_idx >= len(path): target_idx = len(path) - 1
        tx, ty = path[target_idx]
        
        alpha = math.atan2(ty - y, tx - x) - yaw
        while alpha > math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi
        
        delta = math.atan2(2 * 0.33 * math.sin(alpha) / lookahead, 1.0)
        return float(np.clip(delta, -0.55, 0.55)), speed_limit

    def pure_pursuit_adaptive(self, pose, path, min_ld=0.6, max_ld=1.5, cornering_sensitive=True):
        x, y, yaw_deg = pose
        yaw = np.radians(yaw_deg)
        path_np = np.array(path)
        dx = path_np[:,0] - x
        dy = path_np[:,1] - y
        closest_idx = np.argmin(np.hypot(dx, dy))
        
        current_speed = max(self.target_speed, 1.0)
        lookahead = np.clip(0.5 + 0.4 * current_speed, min_ld, max_ld)
        
        # [Modified] 특정 구간(x: 3.25 ~ 7.7, y: 9.0 ~ 10.0)에서 Lookahead 증가
        if (3.25 < x < 7.7) and (8.5 < y < 10.0):
            lookahead = 0.5 # 강제로 멀리 봄
        
        target_idx = closest_idx
        for i in range(closest_idx, len(path)):
            if np.hypot(path[i][0] - x, path[i][1] - y) > lookahead:
                target_idx = i
                break
        
        tx, ty = path[target_idx]
        self.publish_target_marker(tx, ty)
        
        alpha = math.atan2(ty - y, tx - x) - yaw
        while alpha > math.pi: alpha -= 2 * math.pi
        while alpha < -math.pi: alpha += 2 * math.pi
        
        eff_ld = max(np.hypot(tx - x, ty - y), 0.1)
        delta = math.atan2(2 * 0.325 * math.sin(alpha) / eff_ld, 1.0)
        
        # [Modified] 특정 구간(x: 3.25 ~ 7.7, y: 9.0 ~ 10.0)에서 조향 게인 증가
        if (3.25 < x < 7.7) and (8.5 < y < 10.0):
            delta *= 1.4 # 조향각 30% 증폭
            
        angle = float(np.clip(delta, -0.55, 0.55))
        
        if abs(angle) > 0.3: speed = 2.2
        elif abs(angle) > 0.15: speed = 3.5
        else: speed = 5.0 
            
        return angle, speed

    def get_front_distance(self, lidar, fov=30):
        if len(lidar) == 0: return 10.0
        center = len(lidar) // 2
        span = int(len(lidar) * fov / 180 / 2)
        return np.nanmin(lidar[center - span : center + span]) if len(lidar) > 0 else 10.0

def shutdown_handler(control_pub):
    stop_msg = AckermannDrive()  
    stop_msg.speed = 0; stop_msg.steering_angle = 0
    for _ in range(5): 
        control_pub.publish(stop_msg)
        rospy.sleep(0.1)  

if __name__ == "__main__":
    db_instance = Database(lidar=True)
    test_brain = Brain(db=db_instance)
    rate = rospy.Rate(param.thread_rate)
    control_pub = rospy.Publisher('/drive', AckermannDrive, queue_size=1)
    logger = BrainLogger(test_brain)
    rospy.on_shutdown(lambda: shutdown_handler(control_pub))

    while not rospy.is_shutdown():
        car_angle, car_speed = test_brain.main()
        motor_msg = AckermannDrive()
        motor_msg.steering_angle = car_angle
        motor_msg.speed = car_speed
        control_pub.publish(motor_msg)
        logger.log_data()
        rate.sleep()
