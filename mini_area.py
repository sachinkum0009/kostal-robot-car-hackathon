import sys
import os

# Assuming the "picar.py" file is in the parent directory (..)
folder_path = os.path.abspath("..")  # Get the absolute path to the parent directory

# Add the folder path to sys.path
sys.path.append(folder_path)

import time
import threading
from picar import Picar
from enum import Enum

from picamera2 import Picamera2
from pyzbar import pyzbar
from pyzbar.pyzbar import Decoded
from typing import List
import cv2

# Initialize camera
RESOLUTION = (640, 480)  # 4:3 resolution (max 2592x1944)

class Direction(Enum):
    FORWARD = 1
    BACKWARD = 2
    RIGHT_FORWARD   = 3
    RIGHT = 4
    LEFT_FORWARD    = 5
    LEFT = 6
    CROSSING = 7
    STOP = 8

class MiniCar:
    def __init__(self):
        self.pc = Picar()
        self.direction = Direction.FORWARD
        self.action_thread = threading.Thread(target=self.take_action)
        self.crossing_count = 0

        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(main={"size": RESOLUTION}))

        self.picam2.start()

        self.camera_thread = threading.Thread(target=self.take_image)

    def capture_qr_codes(self) -> List[Decoded]:
        """Capture a frame and return any QR codes present."""
        frame = self.picam2.capture_array()
        qr_codes = pyzbar.decode(frame)
        return qr_codes, frame

    def process_qr_codes(self, qr_codes: List[Decoded]) -> None:
        """Process and print the data from the QR codes."""
        for qr_code in qr_codes:
            data = qr_code.data.decode("utf-8")
            print(f"Detected QR code: {data}")
            return data


    def get_direction_state(self):
        state = self.read_sensor()
        # state is [False, False, True, False, False]
        print(state)
        print(self.direction)
        if state == [0, 0, 1, 0, 0]:
            self.direction = Direction.FORWARD
            self.crossing_count = 0
        elif state == [1, 1, 1, 1, 1]:
            self.crossing_count += 1
            print(f"Crossing line detected {self.crossing_count} times.")
            if self.crossing_count >= 5:  # Stop after 3 crossing lines
                print("Encountered 5 crossing lines. Stopping the robot.")
                self.direction = Direction.STOP
                sys.exit()  # Exit the program
            else:
                self.direction = Direction.CROSSING
        elif state == [0, 1, 1, 0, 0]:
            self.direction = Direction.RIGHT_FORWARD
            self.crossing_count = 0
        elif state == [0, 1, 0, 0, 0]:
            self.direction = Direction.RIGHT_FORWARD
            self.crossing_count = 0
        elif state == [1, 1, 0, 0, 0]:
            self.direction = Direction.RIGHT
            self.crossing_count = 0
        elif state == [1, 0, 0, 0, 0]:
            self.direction = Direction.RIGHT
            self.crossing_count = 0
        elif state == [0, 0, 1, 1, 0]:
            self.direction = Direction.LEFT_FORWARD
            self.crossing_count = 0
        elif state == [0, 0, 0, 1, 0]:
            self.direction = Direction.LEFT_FORWARD
            self.crossing_count = 0
        elif state == [0, 0, 0, 1, 1]:
            self.direction = Direction.LEFT
            self.crossing_count = 0
        elif state == [0, 0, 0, 0, 1]:
            self.direction = Direction.LEFT
            self.crossing_count = 0
    
    def forward(self, speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, True)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, True)
        self.pc.set_speed(self.pc.MOTOR_LEFT, speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, speed)

    def left(self, speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, True)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, False)
        self.pc.set_speed(self.pc.MOTOR_LEFT, speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, speed)

    def right(self, speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, False)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, True)
        self.pc.set_speed(self.pc.MOTOR_LEFT, speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, speed)

    def left_forward(self, left_speed, right_speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, True)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, True)
        self.pc.set_speed(self.pc.MOTOR_LEFT, left_speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, right_speed)

    def right_forward(self, left_speed, right_speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, True)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, True)
        self.pc.set_speed(self.pc.MOTOR_LEFT, left_speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, right_speed)

    def backward(self, speed):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, False)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, False)
        self.pc.set_speed(self.pc.MOTOR_LEFT, speed)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, speed)

    def stop(self):
        self.pc.set_motor_direction(self.pc.MOTOR_LEFT, False)
        self.pc.set_motor_direction(self.pc.MOTOR_RIGHT, False)
        self.pc.set_speed(self.pc.MOTOR_LEFT, 0)
        self.pc.set_speed(self.pc.MOTOR_RIGHT, 0)
    
    def read_sensor(self) -> list:
        return self.pc.get_line_sensor_states()

    def print_direction_state(self):
        if self.direction == Direction.FORWARD:
            print('direction is forward')
        elif self.direction == Direction.BACKWARD:
            print('direction is backward')
        elif self.direction == Direction.LEFT:
            print('direction is left')
        elif self.direction == Direction.RIGHT:
            print('direction is right')
        elif self.direction == Direction.STOP:
            print('direction is stop')
        elif self.direction == Direction.LEFT_FORWARD:
            print('direction is left forward')
        elif self.direction == Direction.RIGHT_FORWARD:
            print('direction is right forward')
        elif self.direction == Direction.CROSSING:
            print('direction is crossing')

    def take_action(self):
        while True:
            if self.crossing_count == 6:
                break
            if self.direction == Direction.FORWARD:
                print('direction is forward')
                self.forward(0.4)
            elif self.direction == Direction.BACKWARD:
                print('direction is backward')
            elif self.direction == Direction.LEFT:
                print('direction is left')
                # self.left(0.4)
                self.left(0.4)
            elif self.direction == Direction.RIGHT:
                print('direction is right')
                self.right(0.4)
            elif self.direction == Direction.STOP:
                print('direction is stop')
                # self.stop()
            elif self.direction == Direction.LEFT_FORWARD:
                print('direction is left forward')
                # self.left_forward(0.2, 0.0)
                self.left_forward(0.2, 0.3)
            elif self.direction == Direction.RIGHT_FORWARD:
                print('direction is right forward')
                # self.right_forward(0.0, 0.2)
                self.right_forward(0.3, 0.2)
            elif self.direction == Direction.CROSSING:
                print('direction is crossing')
            else:
                print("nothing")
            if self.crossing_count == 3:
                break
            time.sleep(0.1)
        
    def take_sensor(self):
        while True:
            self.get_direction_state()
            time.sleep(0.1)

    def find_cx(self, img):
        img[0:360] = 0

        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Apply a binary threshold to isolate black areas
        _, binary = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        # Detect edges using Canny
        edges = cv2.Canny(binary, 100, 200)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Check if any contours are found
        if contours:
            # Find the largest contour (assuming it represents the line)
            largest_contour = max(contours, key=cv2.contourArea)

            # Calculate the moments of the contour to find the center point
            M = cv2.moments(largest_contour)
            if M['m00'] != 0:  # To avoid division by zero
                cX = int(M['m10'] / M['m00'])
                cY = int(M['m01'] / M['m00'])
                
                # Draw the center point on the image
                cv2.circle(img, (cX, cY), 5, (0, 0, 255), -1)  # Red color for center point
        print(cX, cY)
        return cX - 240

    def take_image(self):
        while True:
            # Capture and process QR codes
            qr_codes, frame = self.capture_qr_codes()
            if qr_codes is not None:
                data = self.process_qr_codes(qr_codes)
                print(type(data))
                print('data: ', data)
            cX = self.find_cx(frame)
            print('cx:  ', cX)


    def level2(self):
        self.forward(0.5)
        time.sleep(1)
        self.get_direction_state()



    def level0(self):
        self.forward(0.3)
        time.sleep(1)
        self.get_direction_state()
        self.action_thread.start()
        self.take_sensor()

    
    def level1(self):
        self.forward(0.3)
        time.sleep(1)
        self.camera_thread.start()
        self.get_direction_state()
        self.action_thread.start()
        self.take_sensor()

def main():
    car = MiniCar()
    car.level0()
    # car.left_forward(0.4, 0.1)
    # while True:
    #     car.get_direction_state()
    #     car.print_direction_state()
    #     time.sleep(1)

if __name__ == '__main__':
    main()



    

