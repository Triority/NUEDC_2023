import lcd1602a as LCD
import time

LCD.init_lcd()
time.sleep(2)
LCD.print_lcd(0,0,'start')
