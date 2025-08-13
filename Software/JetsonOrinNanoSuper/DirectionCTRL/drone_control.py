"""
drone_control.py
드론의 방향 제어 및 이동을 관리하는 모듈
"""

import asyncio
import logging
import math
from drone_connection import DroneConnection

class DroneController:
    """드론 방향 제어 클래스"""
    
    def __init__(self, connection: DroneConnection):
        """
        DroneController 초기화
        
        Args:
            connection (DroneConnection): 드론 연결 객체
        """
        self.connection = connection
        self.drone = connection.get_drone_instance()
        self.logger = logging.getLogger(__name__)
        
    async def move_forward(self, distance_m=1.0, speed_m_s=1.0):
        """
        전진 이동
        
        Args:
            distance_m (float): 이동 거리 (미터)
            speed_m_s (float): 이동 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"전진 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 전진은 북쪽(+X) 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[speed_m_s, 0.0, 0.0, 0.0]
            )
            
            # 거리만큼 이동하는 시간 계산
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            # 정지
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"전진 이동 실패: {e}")
            return False
    
    async def move_backward(self, distance_m=1.0, speed_m_s=1.0):
        """
        후진 이동
        
        Args:
            distance_m (float): 이동 거리 (미터)
            speed_m_s (float): 이동 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"후진 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 후진은 남쪽(-X) 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[-speed_m_s, 0.0, 0.0, 0.0]
            )
            
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"후진 이동 실패: {e}")
            return False
    
    async def move_left(self, distance_m=1.0, speed_m_s=1.0):
        """
        좌측 이동
        
        Args:
            distance_m (float): 이동 거리 (미터)
            speed_m_s (float): 이동 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"좌측 이동 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 좌측은 서쪽(-Y) 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, -speed_m_s, 0.0, 0.0]
            )
            
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"좌측 이동 실패: {e}")
            return False
    
    async def move_right(self, distance_m=1.0, speed_m_s=1.0):
        """
        우측 이동
        
        Args:
            distance_m (float): 이동 거리 (미터)
            speed_m_s (float): 이동 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"우측 이동 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 우측은 동쪽(+Y) 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, speed_m_s, 0.0, 0.0]
            )
            
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"우측 이동 실패: {e}")
            return False
    
    async def move_up(self, distance_m=1.0, speed_m_s=0.5):
        """
        상승
        
        Args:
            distance_m (float): 상승 거리 (미터)
            speed_m_s (float): 상승 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"상승 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 상승은 -Z 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, 0.0, -speed_m_s, 0.0]
            )
            
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"상승 실패: {e}")
            return False
    
    async def move_down(self, distance_m=1.0, speed_m_s=0.5):
        """
        하강
        
        Args:
            distance_m (float): 하강 거리 (미터)
            speed_m_s (float): 하강 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"하강 {distance_m}m (속도: {speed_m_s}m/s)")
            
            # NED 좌표계에서 하강은 +Z 방향
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, 0.0, speed_m_s, 0.0]
            )
            
            duration = distance_m / speed_m_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"하강 실패: {e}")
            return False
    
    async def rotate_yaw(self, angle_deg=90.0, angular_speed_deg_s=30.0):
        """
        Yaw 회전 (좌우 회전)
        
        Args:
            angle_deg (float): 회전 각도 (양수: 시계방향, 음수: 반시계방향)
            angular_speed_deg_s (float): 회전 속도 (deg/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            direction = "시계방향" if angle_deg > 0 else "반시계방향"
            self.logger.info(f"{direction} {abs(angle_deg)}도 회전 (속도: {angular_speed_deg_s}deg/s)")
            
            # 회전 방향 결정
            yaw_rate = angular_speed_deg_s if angle_deg > 0 else -angular_speed_deg_s
            
            # NED 좌표계에서 yaw 회전
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, 0.0, 0.0, yaw_rate]
            )
            
            # 회전 시간 계산
            duration = abs(angle_deg) / angular_speed_deg_s
            await asyncio.sleep(duration)
            
            await self.stop()
            return True
            
        except Exception as e:
            self.logger.error(f"Yaw 회전 실패: {e}")
            return False
    
    async def go_to_location(self, latitude_deg, longitude_deg, altitude_m, speed_m_s=5.0):
        """
        특정 GPS 좌표로 이동
        
        Args:
            latitude_deg (float): 목표 위도
            longitude_deg (float): 목표 경도
            altitude_m (float): 목표 고도 (미터)
            speed_m_s (float): 이동 속도 (m/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info(f"GPS 좌표로 이동: ({latitude_deg}, {longitude_deg}, {altitude_m}m)")
            
            # 현재 위치의 Yaw 각도 가져오기
            current_yaw = 0.0
            async for attitude in self.drone.telemetry.attitude_euler():
                current_yaw = attitude.yaw_deg
                break
            
            await self.drone.action.goto_location(
                latitude_deg=latitude_deg,
                longitude_deg=longitude_deg,
                absolute_altitude_m=altitude_m,
                yaw_deg=current_yaw
            )
            
            self.logger.info("목표 위치로 이동 명령 전송 완료")
            return True
            
        except Exception as e:
            self.logger.error(f"GPS 좌표 이동 실패: {e}")
            return False
    
    async def set_velocity_body(self, forward_m_s=0.0, right_m_s=0.0, down_m_s=0.0, yaw_deg_s=0.0):
        """
        기체 좌표계 기준 속도 제어
        
        Args:
            forward_m_s (float): 전진 속도 (m/s)
            right_m_s (float): 우측 속도 (m/s)
            down_m_s (float): 하강 속도 (m/s)
            yaw_deg_s (float): Yaw 회전 속도 (deg/s)
        """
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            await self.drone.offboard.set_velocity_body(
                velocity_body_m_s=[forward_m_s, right_m_s, down_m_s, yaw_deg_s]
            )
            return True
            
        except Exception as e:
            self.logger.error(f"속도 설정 실패: {e}")
            return False
    
    async def hold_position(self):
        """현재 위치에서 호버링"""
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("현재 위치에서 호버링")
            await self.drone.action.hold()
            return True
            
        except Exception as e:
            self.logger.error(f"호버링 실패: {e}")
            return False
    
    async def stop(self):
        """드론 정지 (속도를 0으로 설정)"""
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, 0.0, 0.0, 0.0]
            )
            self.logger.info("드론 정지")
            return True
            
        except Exception as e:
            self.logger.error(f"정지 실패: {e}")
            return False
    
    async def start_offboard_mode(self):
        """Offboard 모드 시작"""
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("Offboard 모드를 시작합니다...")
            
            # 초기 설정값 전송 (Offboard 모드 활성화 전 필요)
            await self.drone.offboard.set_velocity_ned(
                velocity_ned_m_s=[0.0, 0.0, 0.0, 0.0]
            )
            
            # Offboard 모드 시작
            await self.drone.offboard.start()
            self.logger.info("Offboard 모드 시작 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 시작 실패: {e}")
            return False
    
    async def stop_offboard_mode(self):
        """Offboard 모드 종료"""
        if not self.connection.is_connected:
            self.logger.error("드론이 연결되지 않았습니다.")
            return False
            
        try:
            self.logger.info("Offboard 모드를 종료합니다...")
            await self.drone.offboard.stop()
            self.logger.info("Offboard 모드 종료 완료!")
            return True
            
        except Exception as e:
            self.logger.error(f"Offboard 모드 종료 실패: {e}")
            return False


# 메인 실행 예제
async def main():
    """사용 예제"""
    # 드론 연결
    connection = DroneConnection("serial:///dev/ttyTHS0:115200")
    
    if await connection.connect():
        # 컨트롤러 생성
        controller = DroneController(connection)
        
        # 텔레메트리 데이터 확인
        telemetry = await connection.get_telemetry_data()
        if telemetry:
            print(f"현재 드론 상태: {telemetry}")
        
        # 드론 시동
        await connection.arm()
        await asyncio.sleep(2)
        
        # 이륙
        await connection.takeoff(altitude=2.0)
        await asyncio.sleep(5)
        
        # Offboard 모드 시작 (정밀 제어를 위해)
        await controller.start_offboard_mode()
        
        # 방향 제어 테스트
        await controller.move_forward(distance_m=2.0, speed_m_s=1.0)
        await asyncio.sleep(1)
        
        await controller.rotate_yaw(angle_deg=90, angular_speed_deg_s=30)
        await asyncio.sleep(1)
        
        await controller.move_right(distance_m=2.0, speed_m_s=1.0)
        await asyncio.sleep(1)
        
        # Offboard 모드 종료
        await controller.stop_offboard_mode()
        
        # 착륙
        await connection.land()
        await asyncio.sleep(5)
        
        # 시동 끄기
        await connection.disarm()
        
    else:
        print("드론 연결 실패!")

if __name__ == "__main__":
    asyncio.run(main())
