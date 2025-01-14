import timeit

# Original functions (import or define them with prefix "original_")
from launch_angle_calculator import (
    find_camera_distance as original_find_camera_distance,
    find_distance_x as original_find_distance_x,
    find_distance_y as original_find_distance_y,
    find_servo_pan_angle as original_find_servo_pan_angle,
    find_servo_tilt_angle as original_find_servo_tilt_angle,
)
# Optimized functions (import or define them with prefix "optimized_")
from launch_angle_calculator import (
    find_camera_distance as optimized_find_camera_distance,
    find_distance_x as optimized_find_distance_x,
    find_distance_y as optimized_find_distance_y,
    find_servo_pan_angle as optimized_find_servo_pan_angle,
    find_servo_tilt_angle as optimized_find_servo_tilt_angle,
)

# Sample inputs
camera_spacing = 1.5
theta, phi = 34, 50
tilt_angle = 10
initial_velocity = 22


# Wrapping original code in a function
def run_original():
    camera_distance = original_find_camera_distance(camera_spacing, theta, phi)
    distance_x = original_find_distance_x(camera_spacing, camera_distance, phi)
    distance_y = original_find_distance_y(camera_distance, tilt_angle)
    original_find_servo_pan_angle(distance_x, camera_distance, phi)
    original_find_servo_tilt_angle(distance_x, distance_y, initial_velocity)


# Wrapping optimized code in a function
def run_optimized():
    camera_distance = optimized_find_camera_distance(camera_spacing, theta, phi)
    distance_x = optimized_find_distance_x(camera_spacing, camera_distance, phi)
    distance_y = optimized_find_distance_y(camera_distance, tilt_angle)
    optimized_find_servo_pan_angle(distance_x, camera_distance, phi)
    optimized_find_servo_tilt_angle(distance_x, distance_y, initial_velocity)


# Measure average execution time over n runs
def measure_average_time(func, runs=5, iterations=1000):
    times = []
    for _ in range(runs):
        elapsed = timeit.timeit(func, number=iterations)
        times.append(elapsed)
    average_time = sum(times) / len(times)
    return average_time


# Define number of runs and iterations
runs = 100
iterations = 1000

# Get average times
average_original_time = measure_average_time(run_original, runs, iterations)
average_optimized_time = measure_average_time(run_optimized, runs, iterations)

# Display results
print(f"Average Original Code Time: {average_original_time:.6f} seconds over {runs} runs")
print(f"Average Optimized Code Time: {average_optimized_time:.6f} seconds over {runs} runs")
print(f"Speedup: {average_original_time / average_optimized_time:.2f}x faster")
