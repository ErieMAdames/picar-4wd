import picar_4wd as pc4
try:
    while True:
        angle = float(input())
        pc4.get_distance_at(angle)
except KeyboardInterrupt:
    exit()