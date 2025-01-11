import math
import cv2
import mediapipe as mp

FOV = 82.1
AVERAGE_HIP_WIDTH = 32.5  # Average hip width in cm
CAMERA_ID = 1


# Function to calculate focal length in pixels from the field of view and resolution width
def calculate_focal_length_in_pixels(resolution_width, FOV_degrees):
    FOV_radians = FOV_degrees * (math.pi / 180)
    focal_length_pixels = resolution_width / (2 * math.tan(FOV_radians / 2))
    return focal_length_pixels


def estimate_position_and_distance(image_width, image_height, landmark1, landmark2, focal_length, known_width):
    # Calculate the pixel distance between landmarks
    pixel_distance = math.hypot((landmark1.x - landmark2.x) * image_width, (landmark1.y - landmark2.y) * image_height)

    # Calculate distance from the camera
    distance = (known_width * focal_length) / pixel_distance

    # Calculate average position (central point between the hips in image coordinates)
    avg_x = (landmark1.x + landmark2.x) / 2 * image_width
    avg_y = (landmark1.y + landmark2.y) / 2 * image_height

    # Calculate horizontal offset from the center of the image in pixels
    center_x = image_width / 2
    pixel_offset = avg_x - center_x

    # Assuming a constant real-world field of view, calculate offset in cm (approximation)
    distance_per_pixel = math.tan(math.radians(FOV / 2)) * 2 * distance / image_width
    cm_offset = pixel_offset * distance_per_pixel

    return cm_offset, distance, (int(avg_x), int(avg_y))


if __name__ == "__main__":
    # Initialize Mediapipe
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    # Initialize webcam
    cap = cv2.VideoCapture(CAMERA_ID)

    cap.set(cv2.CAP_PROP_FPS, 30)

    while True:
        # Read frames from the webcam
        ret, img = cap.read()
        if not ret or img is None:
            print("Error reading frame from webcam.")
            break

        w = img.shape[1]
        h = img.shape[0]

        # Convert the image to RGB and process it
        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(rgb_img)

        # If no people are detected, skip this frame
        if pose_results.pose_landmarks is None:
            continue

        # Calculate focal length in pixels
        focal_length = calculate_focal_length_in_pixels(w, FOV)

        # Get hip positions
        left_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]

        cm_offset, distance, center_point = estimate_position_and_distance(
            w, h, left_hip, right_hip, focal_length, AVERAGE_HIP_WIDTH
        )

        distance_inches = distance * 0.3937

        # Draw landmarks, connections, and the center point
        mp_drawing.draw_landmarks(img, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        cv2.circle(img, center_point, 5, (255, 0, 0), -1)
        cv2.putText(img, f"Offset: {cm_offset:.2f}cm, Distance: {distance_inches:.2f}in",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        # Show the processed image
        cv2.imshow('Processed Image', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        # Stop on Esc or if windows are closed
        k = cv2.waitKey(30) & 0xff
        if k == 27 or cv2.getWindowProperty('Processed Image', cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()
