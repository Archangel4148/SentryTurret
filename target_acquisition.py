import cv2
import mediapipe as mp
import numpy as np
import serial

from launch_angle_calculator_optimized import find_camera_distance, find_distance_x, find_distance_y, \
    find_servo_pan_angle, find_servo_tilt_angle

# Constants for camera's angular FOVs
HORIZONTAL_FOV = (85.99, 70.43)  # Horizontal FoV in degrees (calculated earlier)
VERTICAL_FOV = (70.09, 43.31)  # Vertical FoV in degrees (calculated earlier)
CAM_1 = 1
CAM_2 = 2

# Setup parameters
CAMERA_SPACING = 0.23
LAUNCH_VELOCITY = 22

# Video feed settings
SHOW_CAMERA_FEED = False
DRAW_POSE_LANDMARKS = True
DRAW_TARGET_POINT = True
STEREOSCOPIC_MODE = False  # Set to True for two cameras, False for one camera

# Serial settings
SERIAL_ENABLE = False
SEND_INTERVAL = 1 / 30  # Time to wait before sending the next target position (in seconds)


def send_serial_data(serial, target_x_px, target_y_px, screen_width):
    red_value = int(255 * (1 - (target_x_px / screen_width)))  # Increase red left
    blue_value = int(255 * (target_x_px / screen_width))  # Increase blue right
    hex_color = f"#{red_value:02x}00{blue_value:02x}"

    # Send the hex color to the Arduino
    serial.write((hex_color + '\n').encode())


def calculate_camera_angle(target_x_px, target_y_px, w, h, hfov, vfov, invert_x=False):
    # Normalize the pixel positions
    x_norm = target_x_px / w
    if invert_x:
        x_norm = 1 - x_norm
    y_norm = 1 - target_y_px / h

    # Calculate the pan (horizontal) angle using the horizontal FOV
    hidden_angle = 0.5 * (180 - hfov)
    pan_angle = hidden_angle + (x_norm * hfov)

    # Calculate the tilt (vertical) angle using the vertical FOV
    hidden_angle = 0.5 * (180 - vfov)
    tilt_angle = hidden_angle + (y_norm * vfov)

    return pan_angle, tilt_angle


def process_frame(frame, pose, mp_drawing, mp_pose, hfov, vfov, serial=None, invert_x=False):
    """Processes a single frame, performs pose detection, and calculates angles."""
    # Convert the image to RGB and process it
    rgb_img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pose_results = pose.process(rgb_img)

    # If no people are detected, return
    if pose_results.pose_landmarks is None:
        return frame, None, None

    h, w, _ = frame.shape

    if SHOW_CAMERA_FEED:
        # Use the image from the webcam
        result_image = frame
    else:
        # Use a black background
        result_image = np.zeros(shape=[h, w, 3], dtype=np.uint8)  # Black background

    if DRAW_POSE_LANDMARKS:
        # Draw landmarks and connections (wireframe)
        mp_drawing.draw_landmarks(
            result_image,
            pose_results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS,
            landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
            connection_drawing_spec=mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2),
        )

    # Get positions of shoulders and hips
    left_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
    right_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
    left_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
    right_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]

    # Center of the shoulders
    shoulder_x = (left_shoulder.x + right_shoulder.x) / 2
    shoulder_y = (left_shoulder.y + right_shoulder.y) / 2

    # Center of the hips
    hip_x = (left_hip.x + right_hip.x) / 2
    hip_y = (left_hip.y + right_hip.y) / 2

    # Find heart position
    heart_x = (shoulder_x + hip_x) / 2
    heart_y = shoulder_y * 0.8 + hip_y * 0.2  # Weighted average to shift point up

    # Convert to pixel coordinates
    target_x_px = int(heart_x * w)
    target_y_px = int(heart_y * h)

    if DRAW_TARGET_POINT:
        # Draw a red dot at the target position
        cv2.circle(result_image, (target_x_px, target_y_px), 5, (0, 0, 255), -1)

    # Calculate the servo angles
    pan_angle, tilt_angle = calculate_camera_angle(target_x_px, target_y_px, w, h, hfov, vfov, invert_x)

    # Send data to Arduino
    if SERIAL_ENABLE and serial is not None:
        send_serial_data(serial, target_x_px, target_y_px, w)

    return result_image, pan_angle, tilt_angle


def main():
    if SERIAL_ENABLE:
        # Initialize serial communication
        try:
            ser = serial.Serial('COM7', 115200)
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            exit(1)
    else:
        ser = None

    # Initialize Mediapipe
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    # Initialize webcams based on mode
    cap1 = cv2.VideoCapture(CAM_1)  # First webcam
    cap2 = cv2.VideoCapture(CAM_2) if STEREOSCOPIC_MODE else None  # Second webcam if stereoscopic mode is enabled

    cap1.set(cv2.CAP_PROP_FPS, 30)
    if STEREOSCOPIC_MODE:
        cap2.set(cv2.CAP_PROP_FPS, 30)

    if not cap1.isOpened():
        print("Error: Cannot open webcam 0.")
        exit(1)
    if STEREOSCOPIC_MODE and (cap2 is None or not cap2.isOpened()):
        print("Error: Cannot open webcam 1.")
        exit(1)

    try:
        while True:
            # Read frames from the first webcam
            ret1, img1 = cap1.read()
            if not ret1 or img1 is None:
                print("Error reading frame from webcam 0.")
                break

            # Process frames from the first webcam
            result_image1, pan_angle1, tilt_angle1 = process_frame(img1, pose, mp_drawing, mp_pose, HORIZONTAL_FOV[0],
                                                                   VERTICAL_FOV[0], ser)

            print(f"Webcam 0 -> Pan: {pan_angle1}°, Tilt: {tilt_angle1}°")
            cv2.imshow('Webcam 0 Pose Detection', result_image1)

            if STEREOSCOPIC_MODE:
                # Read frames from the second webcam
                ret2, img2 = cap2.read()
                if not ret2 or img2 is None:
                    print("Error reading frame from webcam 1.")
                    break

                # Process frames from the second webcam
                result_image2, pan_angle2, tilt_angle2 = process_frame(img2, pose, mp_drawing, mp_pose,
                                                                       HORIZONTAL_FOV[1], VERTICAL_FOV[1], ser, True)
                # print(f"Webcam 1 -> Pan: {pan_angle2}°, Tilt: {tilt_angle2}°\n")
                cv2.imshow('Webcam 1 Pose Detection', result_image2)

                # Process frames from both webcams
                if pan_angle1 is not None and pan_angle2 is not None and tilt_angle1 is not None and tilt_angle2 is not None:
                    camera_distance = find_camera_distance(CAMERA_SPACING, abs(pan_angle1), abs(pan_angle2))
                    distance_x = find_distance_x(CAMERA_SPACING, camera_distance, abs(pan_angle2))
                    distance_y = find_distance_y(camera_distance, tilt_angle1)

                    servo_pan_angle, actual_pan_angle = find_servo_pan_angle(distance_x, camera_distance, pan_angle2)
                    servo_tilt_angle, actual_tilt_angle = find_servo_tilt_angle(distance_x, distance_y, LAUNCH_VELOCITY)

                    # Print angles rounded to 3 decimal places
                    # print(f"Angles: Pan: {pan_angle1:.3f}°, Tilt: {tilt_angle1:.3f}° - Pan: {pan_angle2:.3f}°, Tilt: {tilt_angle2:.3f}°")

                    # print(f"Camera Distance: {camera_distance:.3f}m")
                    print(f"Gun Distance - X: {distance_x:.3f}m, Y: {distance_y:.3f}m")
                    # print(f"Actual Pan Angle: {actual_pan_angle:.3f}°")
                    # print(f"Servo Pan Angle: {servo_pan_angle}°")
                    # print(f"Actual Tilt Angle: {actual_tilt_angle:.3f}°")
                    # print(f"Servo Tilt Angle: {servo_tilt_angle}°")

            # Stop on Esc or if windows are closed
            k = cv2.waitKey(30) & 0xff
            if k == 27 or cv2.getWindowProperty('Webcam 0 Pose Detection', cv2.WND_PROP_VISIBLE) < 1 or \
                    (STEREOSCOPIC_MODE and cv2.getWindowProperty('Webcam 1 Pose Detection', cv2.WND_PROP_VISIBLE) < 1):
                break
    finally:
        cap1.release()
        if STEREOSCOPIC_MODE and cap2:
            cap2.release()
        cv2.destroyAllWindows()

        if SERIAL_ENABLE and ser is not None:
            ser.write(b'#000000\n')
            ser.close()


if __name__ == "__main__":
    main()
