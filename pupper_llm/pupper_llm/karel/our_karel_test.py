# karel_test.py

import karel


def main():
    pupper = karel.KarelPupper()

    pupper.move()
    pupper.turn_left()
    pupper.move()
    pupper.turn_left()
    pupper.move()
    pupper.turn_left()
    pupper.move()
    pupper.turn_left()
    pupper.stop()
    for i in range(10):
        pupper.bark()
    

if __name__ == '__main__':
    main()