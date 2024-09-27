import time
from picar import Picar

from picamera2 import Picamera2
from pyzbar import pyzbar
from pyzbar.pyzbar import Decoded
from typing import List
import threading

# Initialize camera
RESOLUTION = (640, 480)  # 4:3 resolution (max 2592x1944)
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": RESOLUTION}))

# Set controls if required
# picam2.set_controls({"AwbEnable": False, "ExposureTime": 15000, "AnalogueGain": 8})

# Start the camera
picam2.start()

# Initialize the Picar
pc = Picar()

# PD control parameters
Kp = 0.085#0.08 #0.08   #0.08   #0.08   #0.075  #0.075  #0.075  #0.08   #0.07   #0.07   #0.07   #0.05# Proportional gain
Kd = 0.08#0.3   #0.25   #0.2    #0.15   #0.09   #0.085  #0.08   #0.08   #0.08   #0.07   #0.02   #0.01 # Derivative gain

#level 1 best kp = 0.085, kd = 0.08, base_speed = 0.3

base_speed = 0.3 #0.3

# Variables to track error, previous sensor state, and time
previous_error = 0
previous_time = time.time()
previous_sensor_state = [0, 0, 1, 0, 0]  # Assume initial state is forward

data = None

def capture_qr_codes() -> List[Decoded]:
    """Capture a frame and return any QR codes present."""
    frame = picam2.capture_array()
    qr_codes = pyzbar.decode(frame)
    return qr_codes

def process_qr_codes(qr_codes: List[Decoded]) -> None:
    global data
    """Process and print the data from the QR codes."""
    for qr_code in qr_codes:
        data = qr_code.data.decode("utf-8")
        print(f"Detected QR code: {data}")

def calculate_error(sensor_states, previous_sensor_states):
    # Handle the special case where all sensors are off
    # if sensor_states == [0, 0, 0, 0, 0]:
    #     if previous_sensor_states == [0, 0, 0, 0, 1]:
    #         return -5  # Extreme left (turning left level 5)
    #     elif previous_sensor_states == [1, 0, 0, 0, 0]:
    #         return 5  # Extreme right (turning right level 5)
    #     else:
    #         return None  # Line lost and no clear direction
    # Map sensor readings to error values
    if sensor_states == [0, 0, 1, 0, 0]:
        return 0  # Centered on the line, moving forward
    if sensor_states == [1, 1, 1, 1, 1]:
        return 6 # for crossing
    elif sensor_states == [0, 1, 1, 0, 0]:
        return 1  # Slightly to the right (turning right level 1)
    elif sensor_states == [0, 1, 0, 0, 0]:
        return 2  # Further to the right (turning right level 2)
    elif sensor_states == [1, 1, 0, 0, 0]:
        return 3  # Even further to the right (turning right level 3)
    elif sensor_states == [1, 0, 0, 0, 0] or sensor_states == [1, 1, 1, 1, 0]:
        return 4  # Extreme right (turning right level 4)
    elif sensor_states == [1, 1, 1, 0, 0]:
        return 5
    elif sensor_states == [0, 0, 1, 1, 0]:
        return -1  # Slightly to the left (turning left level 1)
    elif sensor_states == [0, 0, 0, 1, 0]:
        return -2  # Further to the left (turning left level 2)
    elif sensor_states == [0, 0, 0, 1, 1]:
        return -3  # Even further to the left (turning left level 3)
    elif sensor_states == [0, 0, 0, 0, 1] or sensor_states == [0, 1, 1, 1, 1]:
        return -4  # Extreme left (turning left level 4)
    elif sensor_states == [0, 0, 1, 1, 1]:
        return -5
    else:
        return None  # Line lost

def pd_controller(error, previous_error, delta_time):
    # Calculate proportional and derivative terms
    proportional = error
    derivative = (error - previous_error) / delta_time

    # PD output
    control_output = Kp * proportional + Kd * derivative

    return control_output

def qr_thread():
    # while True:
        # Capture and process QR codes
    qr_codes = capture_qr_codes()
    if qr_codes is not None:
        process_qr_codes(qr_codes)
        # time.sleep(0.5)

# thread = threading.Thread(target = qr_thread)
# thread.start()

turn_right_flag = False

pc.set_motor_direction(pc.MOTOR_LEFT, True)  # Forward
pc.set_motor_direction(pc.MOTOR_RIGHT, True)  # Forward
pc.set_speed(pc.MOTOR_LEFT, 0.3)
pc.set_speed(pc.MOTOR_RIGHT, 0.3)
time.sleep(1.0)


try:
    while True:
        qr_thread()
        if data == 'left':
            turn_right_flag = True
        current_time = time.time()
        delta_time = current_time - previous_time

        # Read the line sensor states
        sensor_states = pc.get_line_sensor_states()
        print(f"sensors: {sensor_states}")

        # Calculate the error based on sensor readings
        error = calculate_error(sensor_states, previous_sensor_state)

        if error is None:
            error = previous_error

        if error == 6: # crossing
            # exit(0)
            # stop
            # rotate camera
            pc.set_motor_direction(pc.MOTOR_LEFT, False)  # Forward
            pc.set_motor_direction(pc.MOTOR_RIGHT, True)  # Forward
            pc.set_speed(pc.MOTOR_LEFT, 0)
            pc.set_speed(pc.MOTOR_RIGHT, 0)
            while data is None:
                pc.set_camera_angle(45)
                time.sleep(0.5)
                # exit(0)
                qr_thread()
                if data is not None:
                    break
                pc.set_camera_angle(-45)
                time.sleep(0.5)
                qr_thread()
                if data is not None:
                    break
                pc.set_camera_angle(-60)
                qr_thread()
                if data is not None:
                    break

            time.sleep(0.5)
            pc.set_motor_direction(pc.MOTOR_LEFT, True)  # Forward
            pc.set_motor_direction(pc.MOTOR_RIGHT, True)  # Forward
            pc.set_speed(pc.MOTOR_LEFT, 0.2)
            pc.set_speed(pc.MOTOR_RIGHT, 0.2) 
            time.sleep(0.5)
            if "left" in data:
                turn_right_flag = False
                # turn right
                # Set motor directions and speeds
                pc.set_motor_direction(pc.MOTOR_LEFT, False)  # Forward
                pc.set_motor_direction(pc.MOTOR_RIGHT, True)  # Forward
                pc.set_speed(pc.MOTOR_LEFT, 0.3)
                pc.set_speed(pc.MOTOR_RIGHT, 0.3)
                time.sleep(2.0)
                continue
            elif 'right' in data:
                turn_right_flag = False
                pc.set_motor_direction(pc.MOTOR_LEFT, True)   # Forward
                pc.set_motor_direction(pc.MOTOR_RIGHT, False)   # Forward
                pc.set_speed(pc.MOTOR_LEFT, 0.3)
                pc.set_speed(pc.MOTOR_RIGHT, 0.3)
                time.sleep(2.0)
                continue


        # Calculate PD controller output
        control_output = pd_controller(error, previous_error, delta_time)
            
        # Adjust motor speeds based on control output
        left_motor_speed = base_speed + control_output
        right_motor_speed = base_speed - control_output
            
        # Ensure motor speeds are within a valid range
        left_motor_speed = max(0, min(1, left_motor_speed))
        right_motor_speed = max(0, min(1, right_motor_speed))
            
        # Set motor directions and speeds
        pc.set_motor_direction(pc.MOTOR_LEFT, True)  # Forward
        pc.set_motor_direction(pc.MOTOR_RIGHT, True)  # Forward
        pc.set_speed(pc.MOTOR_LEFT, left_motor_speed)
        pc.set_speed(pc.MOTOR_RIGHT, right_motor_speed)
            
        # Update previous error, sensor state, and time
        previous_error = error
        previous_sensor_state = sensor_states


        print("data is ", data)

        # Small delay to prevent excessive CPU usage
        time.sleep(0.05)

except KeyboardInterrupt:
    # Handle the Ctrl+C exception to stop the car safely
    pc.stop_motor(pc.MOTOR_LEFT)
    pc.stop_motor(pc.MOTOR_RIGHT)
    print("Program interrupted and terminated.")

finally:
    # Ensure motors are stopped before exiting
    pc.stop_motor(pc.MOTOR_LEFT)
    pc.stop_motor(pc.MOTOR_RIGHT)
    print("Program terminated.")
