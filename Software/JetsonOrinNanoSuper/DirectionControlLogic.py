# drone_controller.py
import time
import math
from dronekit import mavutil

# Adjust the default vertical speed here if needed
VERTICAL_SPEED = 1.0  # m/s

def hover(DataRT, duration):
    """
    Commands the drone to hover in place for a specific duration.
    This is achieved by sending a velocity command with all speeds set to zero.
    """
    msg = DataRT.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    for _ in range(duration):
        DataRT.send_mavlink(msg)
        time.sleep(1)

def execute_flight_command(DataRT, vertical_mode, angle, speed, duration):
    """
    Executes a flight command based on 4 parameters.
    """
    if DataRT.mode.name != "GUIDED":
        print("Error: Vehicle must be in GUIDED mode.")
        return

    # Translate vertical_mode string into a numerical vz_speed
    if vertical_mode == 'ascend':
        vz_speed = -VERTICAL_SPEED
    elif vertical_mode == 'descend':
        vz_speed = VERTICAL_SPEED
    else: # 'level'
        vz_speed = 0.0

    # Calculate horizontal velocities
    angle_rad = math.radians(angle)
    vx = speed * math.cos(angle_rad)
    vy = speed * math.sin(angle_rad)
    
    # Create and send the MAVLink message
    msg = DataRT.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, 0, 0, 0, vx, vy, vz_speed, 0, 0, 0, 0, 0)

    for _ in range(duration):
        DataRT.send_mavlink(msg)
        time.sleep(1)

    # Send a stop command to hover in place after the movement
    stop_msg = DataRT.message_factory.set_position_target_local_ned_encode(
        0, 0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    DataRT.send_mavlink(stop_msg)
