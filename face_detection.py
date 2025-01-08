import cv2
import mediapipe as mp

# Initialize Mediapipe solutions
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# Initialize webcam
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Cannot open webcam.")
    exit(1)

# Initialize Mediapipe Pose module
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

while True:
    # Read the frame
    ret, img = cap.read()
    if not ret:
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
        chest_x_px = int(chest_x * w)
        chest_y_px = int(chest_y * h)

        # Draw a red dot at the center of the chest
        cv2.circle(img, (chest_x_px, chest_y_px), 5, (0, 0, 255), -1)

    # Display the result
    cv2.imshow('Mediapipe Pose Detection', img)

    # Stop on Escape key or if window is closed
    k = cv2.waitKey(30) & 0xff
    if k == 27 or cv2.getWindowProperty('Mediapipe Pose Detection', cv2.WND_PROP_VISIBLE) < 1:
        break

cap.release()
cv2.destroyAllWindows()
