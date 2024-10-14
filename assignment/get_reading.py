import picar_4wd as pc4

def main():
    while True:
        input()
        print(pc4.get_distance_at(-10))
        


if __name__ == "__main__":
    try: 
        main()
    except KeyboardInterrupt:
        print('\nStopping')
    finally:
        pc4.stop()
