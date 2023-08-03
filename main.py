from class_driver.class_driver import motor_driver
import time
if __name__ == '__main__':
    left_driver = motor_driver("/dev/left_roll",115200)
    right_driver = motor_driver("/dev/right_roll",115200)
    left_driver.start()
    right_driver.start()
    try:
        location_list = []
        while True:
            r = input("r")
            l = input("l")
            left_driver.velocity = float(r)
            right_driver.velocity = float(l)

    except:
        left_driver.velocity = 0
        time.sleep(0.1)
        exit()