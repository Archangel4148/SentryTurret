import math


def calculate_launch_angle(x, y, v0):
    g = 9.8  # Acceleration due to gravity (m/s^2)

    # Compute the discriminant of the quadratic equation
    discriminant = (v0 ** 4) - g * (g * x ** 2 + 2 * y * v0 ** 2)

    if discriminant < 0:
        return None  # Target is unreachable

    # Calculate the two possible angles
    angle1 = math.atan((v0 ** 2 + math.sqrt(discriminant)) / (g * x))
    angle2 = math.atan((v0 ** 2 - math.sqrt(discriminant)) / (g * x))

    # Convert angles to degrees
    angle1_deg = math.degrees(angle1)
    angle2_deg = math.degrees(angle2)

    return min(angle1_deg, angle2_deg), max(angle1_deg, angle2_deg)


def calculate_laser_angle(x, y):
    return math.degrees(math.atan2(y, x))


# Required inputs
x = float(input("Enter the horizontal distance to the target (m): "))
y = float(input("Enter the height difference to the target (m): "))
v0 = float(input("Enter the launch velocity (m/s): "))

angles = calculate_launch_angle(x, y, v0)
laser_angle = calculate_laser_angle(x, y)

if angles:
    print(f"\nPossible launch angles: {angles[0]:.2f} degrees and {angles[1]:.2f} degrees")
else:
    print("\nThe target is unreachable with the given initial velocity.")

print(f"Laser angle: {laser_angle:.2f} degrees")
