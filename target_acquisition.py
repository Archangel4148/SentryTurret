import cv2
import mediapipe as mp
import numpy as np
import serial

# Constants for camera's angular FOVs
HORIZONTAL_FOV = 85.99  # Horizontal FoV in degrees (calculated earlier)
VERTICAL_FOV = 70.09  # Vertical FoV in degrees (calculated earlier)

# Video feed settings
SHOW_CAMERA_FEED = False
DRAW_POSE_LANDMARKS = True
DRAW_TARGET_POINT = True

# Serial settings
SERIAL_ENABLE = False
SEND_INTERVAL = 1 / 30  # Time to wait before sending the next target position (in seconds)


def send_serial_data(serial, target_x_px, target_y_px, screen_width):
    red_value = int(255 * (1 - (target_x_px / screen_width)))  # Increase red left
    blue_value = int(255 * (target_x_px / screen_width))  # Increase blue right
    hex_color = f"#{red_value:02x}00{blue_value:02x}"

    # Send the hex color to the Arduino
    serial.write((hex_color + '\n').encode())


def calculate_camera_angle(target_x_px, target_y_px, w, h):
    # Normalize the pixel positions
    x_norm = target_x_px / w
    y_norm = target_y_px / h

    # Calculate the pan (horizontal) angle using the horizontal FOV
    pan_angle = (x_norm - 0.5) * HORIZONTAL_FOV

    # Calculate the tilt (vertical) angle using the vertical FOV
    tilt_angle = (y_norm - 0.5) * VERTICAL_FOV

    return pan_angle, tilt_angle


def process_frame(frame, pose, mp_drawing, mp_pose, serial=None):
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
    pan_angle, tilt_angle = calculate_camera_angle(target_x_px, target_y_px, w, h)

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

    # Initialize webcam
    cap = cv2.VideoCapture(0)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Frames per second: {fps}")
    if not cap.isOpened():
        print("Error: Cannot open webcam.")
        exit(1)

    try:
        while True:
            ret, img = cap.read()
            if not ret or img is None:
                print("Error reading frame from webcam.")
                break

            result_image, pan_angle, tilt_angle = process_frame(img, pose, mp_drawing, mp_pose, ser)

            print(f"Pan Angle: {pan_angle}°, Tilt Angle: {tilt_angle}°")

            # Display the image
            cv2.imshow('Mediapipe Pose Detection', result_image)

            # Stop on Esc or if window is closed
            k = cv2.waitKey(30) & 0xff
            if k == 27 or cv2.getWindowProperty('Mediapipe Pose Detection', cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

        if SERIAL_ENABLE and ser is not None:
            ser.write(b'#000000\n')
            ser.close()


if __name__ == "__main__":
    main()
