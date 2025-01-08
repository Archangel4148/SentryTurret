import time
import cv2
import mediapipe as mp
import serial

SEND_INTERVAL = 0.01  # Time to wait before sending the next target position (in seconds)


def send_target_position(serial, target_x_px, target_y_px, screen_width):
    # Normalize target_x_px to get the red (left) to blue (right) range
    red_value = int(255 * (1 - (target_x_px / screen_width)))  # Increase red as it moves left
    blue_value = int(255 * (target_x_px / screen_width))  # Increase blue as it moves right
    green_value = 0  # You can adjust this if you want a color gradient in the green channel

    # Create the hex color string
    hex_color = f"#{red_value:02x}{green_value:02x}{blue_value:02x}"

    # Send the hex color to the Arduino
    serial.write((hex_color + '\n').encode())


def main():
    # Initialize serial communication
    try:
        ser = serial.Serial('COM7', 115200)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        exit(1)

    # Initialize Mediapipe solutions
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils

    # Initialize webcam
    cap = cv2.VideoCapture(0)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"Frames per second: {fps}")
    if not cap.isOpened():
        print("Error: Cannot open webcam.")
        exit(1)

    # Initialize Mediapipe Pose module
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    last_send_time = time.time()  # Track the time of the last send
    elapsed_time = 0  # Accumulator for elapsed time

    while True:
        # Read the frame
        ret, img = cap.read()
        if not ret or img is None:
            print("Error reading frame from webcam.")
            break

        # Convert the image to RGB for Mediapipe
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Process the image for pose detection
        pose_results = pose.process(rgb_img)

        # Draw pose detection results
        if pose_results.pose_landmarks:
            # Draw landmarks and connections
            mp_drawing.draw_landmarks(
                img,
                pose_results.pose_landmarks,
                mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2),
                connection_drawing_spec=mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2),
            )

            # Get positions of the left and right shoulders and hips
            left_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_SHOULDER]
            right_shoulder = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_SHOULDER]
            left_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
            right_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]

            # Calculate the adjusted center of the chest (give more weight to shoulders)
            chest_x = (left_shoulder.x + right_shoulder.x) / 2
            chest_y = (left_shoulder.y + right_shoulder.y + left_hip.y - 0.08) / 3

            # Convert to pixel coordinates
            h, w, _ = img.shape
            target_x_px = int(chest_x * w)
            target_y_px = int(chest_y * h)

            # Draw a red dot at the center of the chest
            cv2.circle(img, (target_x_px, target_y_px), 5, (0, 0, 255), -1)

            # Update elapsed time
            elapsed_time = time.time() - last_send_time

            # Send target position to Arduino if enough time has passed
            if elapsed_time >= SEND_INTERVAL:
                send_target_position(ser, target_x_px, target_y_px, w)
                last_send_time = time.time()  # Reset the last send time

        # Display the result
        cv2.imshow('Mediapipe Pose Detection', img)

        # Stop on Escape key or if window is closed
        k = cv2.waitKey(30) & 0xff
        if k == 27 or cv2.getWindowProperty('Mediapipe Pose Detection', cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()

    # Turn off the LED
    ser.write(b'#000000\n')
    ser.close()


if __name__ == "__main__":
    main()
