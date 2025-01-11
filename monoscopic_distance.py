import math

import cv2
import mediapipe as mp

FOV = 82.1
AVERAGE_HIP_WIDTH = 30.0  # Average hip width in cm (if needed as a reference)
AVERAGE_WAIST_TO_HEAD_DISTANCE = 110.0  # Estimated average distance from waist to head in cm
CAMERA_ID = 1
MAX_VALID_DISTANCE_CM = 610  # 20 feet in cm
MIN_PIXEL_DISTANCE = 25  # Minimum pixel distance to avoid side-view inflation
SIGNIFICANT_DIFFERENCE_THRESHOLD = 1.5  # Threshold to compare the distances (hip vs waist-to-head)


def calculate_focal_length_in_pixels(resolution_width, FOV_degrees):
    FOV_radians = FOV_degrees * (math.pi / 180)
    focal_length_pixels = resolution_width / (2 * math.tan(FOV_radians / 2))
    return focal_length_pixels


def calculate_distance_from_pixel_measurement(pixel_distance, focal_length, known_width):
    return (known_width * focal_length) / pixel_distance


def pixel_distance(landmark1, landmark2, image_width, image_height):
    return math.hypot((landmark1.x - landmark2.x) * image_width, (landmark1.y - landmark2.y) * image_height)


if __name__ == "__main__":
    mp_pose = mp.solutions.pose
    mp_drawing = mp.solutions.drawing_utils
    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    cap = cv2.VideoCapture(CAMERA_ID)
    cap.set(cv2.CAP_PROP_FPS, 30)

    focal_length = None

    while True:
        ret, img = cap.read()
        if not ret or img is None:
            print("Error reading frame from webcam.")
            break

        w = img.shape[1]
        h = img.shape[0]

        if focal_length is None:
            focal_length = calculate_focal_length_in_pixels(w, FOV)

        rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pose_results = pose.process(rgb_img)

        if pose_results.pose_landmarks is None:
            continue

        # Get key landmarks
        left_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.LEFT_HIP]
        right_hip = pose_results.pose_landmarks.landmark[mp_pose.PoseLandmark.RIGHT_HIP]
        head = pose_results.pose_landmarks.landmark[
            mp_pose.PoseLandmark.NOSE]  # Using nose as an approximation for top of head

        # Calculate hip width (horizontal distance between hips)
        hip_width_px = pixel_distance(left_hip, right_hip, w, h)
        distance_from_camera = None
        measurement_used = ""

        if hip_width_px >= MIN_PIXEL_DISTANCE:
            # Calculate distance using hip width
            distance_hip = calculate_distance_from_pixel_measurement(
                hip_width_px, focal_length, AVERAGE_HIP_WIDTH
            )
            measurement_used = "Hip Width"
        else:
            distance_hip = None

        # Calculate waist-to-head distance (vertical distance between hip and head)
        waist_to_head_pixel_dist = pixel_distance(left_hip, head, w, h)
        distance_waist_to_head = calculate_distance_from_pixel_measurement(
            waist_to_head_pixel_dist, focal_length, AVERAGE_WAIST_TO_HEAD_DISTANCE
        )

        # Use the smaller of the two distances if hip measurement is significantly larger than waist-to-head
        if distance_hip and distance_waist_to_head:
            if distance_hip >= (distance_waist_to_head * SIGNIFICANT_DIFFERENCE_THRESHOLD):
                distance_from_camera = distance_waist_to_head
                measurement_used = "Waist to Head"
            else:
                distance_from_camera = min(distance_hip, distance_waist_to_head)
                if distance_hip <= distance_waist_to_head:
                    measurement_used = "Hip Width"
                else:
                    measurement_used = "Waist to Head"
        elif distance_hip:
            distance_from_camera = distance_hip
            measurement_used = "Hip Width"

        elif distance_waist_to_head:
            distance_from_camera = distance_waist_to_head
            measurement_used = "Waist to Head"

        # If the calculated distance exceeds the maximum valid distance, use fallback measurement
        if distance_from_camera is not None and distance_from_camera > MAX_VALID_DISTANCE_CM:
            distance_from_camera = None
            measurement_used = "Exceeded Max Distance"

        # Convert distance from cm to inches (1 inch = 2.54 cm)
        if distance_from_camera is not None:
            distance_inches = distance_from_camera / 2.54  # Convert from cm to inches

        # Display results
        if distance_from_camera is None:
            cv2.putText(img, "Missing Fallback or Insufficient Data", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 0, 255), 2, cv2.LINE_AA)
        else:
            cv2.putText(img, f"Distance: {distance_inches:.2f}in", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1,
                        (0, 255, 0), 2, cv2.LINE_AA)

        # Display which measurement was used
        cv2.putText(img, f"Measurement: {measurement_used}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 0), 2, cv2.LINE_AA)

        mp_drawing.draw_landmarks(img, pose_results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        cv2.imshow('Processed Image', cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

        k = cv2.waitKey(30) & 0xff
        if k == 27 or cv2.getWindowProperty('Processed Image', cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    cv2.destroyAllWindows()
