import picar_4wd as pc4

def main():
    while True:
        current_angle = int(input())
        distance = pc4.get_distance_at(current_angle)
        


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
