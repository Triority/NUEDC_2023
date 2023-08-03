

import smbus
import time
import sys
import lcd1602a as LCD
 
if __name__ == '__main__':  
    LCD.init_lcd()
    time.sleep(1)
    LCD.print_lcd(2, 0, 'WWW.QUWJ.COM')
    for x in range(1, 4):
        LCD.turn_light(0)
        LCD.print_lcd(4, 1, 'LIGHT OFF')
        time.sleep(0.5)
        LCD.turn_light(1)
        LCD.print_lcd(4, 1, 'LIGHT ON ')
        time.sleep(0.5)
 
    LCD.turn_light(0)
     
    while True:
        now = time.strftime('%m/%d %H:%M:%S', time.localtime(time.time()))
        LCD.print_lcd(1, 1, now)
        time.sleep(0.2)
import logging
#print on log file
logging.basicConfig(level=logging.INFO,
                format='%(asctime)s <%(levelname)s> [line:%(lineno)d] %(filename)s : %(message)s',
                datefmt='%Y-%m-%d %H:%M:%S',
                filename='trace.log',
                filemode='a')#w a
#print on screem				
console = logging.StreamHandler()
console.setLevel(logging.DEBUG)
formatter = logging.Formatter('%(asctime)s <%(levelname)s> [line:%(lineno)d] %(filename)s : %(message)s')
console.setFormatter(formatter)
logging.getLogger('').addHandler(console)
if __name__ == '__main__':				
	#CRITICAL > ERROR > WARNING > INFO > DEBUG > NOTSET
	logging.critical('This is critical message')
	logging.error('This is error message')
	logging.warning('This is warning message')
	logging.info('This is info message')
	logging.debug('This is debug message')
	#logging.notset('This is notset message')
