import picar_4wd as fc
import time

def scan_and_avoid(speed, safe_distance):
    while True:
        scans = []
        for angle in range(-90, 91, 30):
            distance = fc.get_distance_at(angle)
            scans.append((angle, distance))
            print(f"Angle: {angle}, Distance: {distance}")
        closest = min(scans, key=lambda x: x[1] if x[1] > -1 else float('inf'))
        print(closest)
        if closest[1] <= safe_distance:
            fc.stop()
            print(f"Obstalce detected at angle {closest[0]} with distance {closest[1]} cm.")
            if closest[0] >= 0:
                print("Turning right")
                fc.backward(speed/2)
                time.sleep(0.5)
                fc.turn_right(speed/2)
            else:
                print("Turning left")
                fc.backward(speed/2)
                time.sleep(0.5)
                fc.turn_left(speed/2)
            time.sleep(0.5)
            fc.stop()
        else:
            fc.forward(speed)


if __name__ == "__main__":
    try:
        speed = 10
        safe_distance = 15
        scan_and_avoid(speed, safe_distance)
    finally:
        fc.stop()
        fc.servo.set_angle(0)
