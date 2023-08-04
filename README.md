# 2023电赛源代码

# 目录文件说明(按代码写完的时间顺序说明)
+ `test`:存放测试使用的程序**(仅供测试)**
  + `serial_communicate.py`:与电机驱动板的串口通信测试
  + `serial_communicate.py`:与电机控制程序的套接字通讯控制测试

+ `socket_driver`:存放电机驱动程序**(已经弃用)**
  + `driver_left.py`:左侧电机驱动程序
  
  使用方法：通过套接字连接向其发送速度数据，格式为`Tx`，`T`为固定标识，`x`为速度，可以为整数或浮点数。端口为`25501`

  + `driver_left.py`:右侧电机驱动程序，使用方法同上，但是端口为`25502`

+ `class_driver`:存放电机调用类目录**(电机控制用这个)**。写完前面的套接字通讯的程序之后，发现写的是个机霸代码，直接调用类不就完了
  + `class_driver.py`:电机调用程序，调用方法如下:
    + `left_driver = motor_driver("/dev/left_roll",115200)`以左侧电机为例首先定义类对象，参数为串口路径和波特率
    + ` left_driver.start()`调用start方法，开始连接并控制电机
      + 之后就可以读取或写入类的变量来控制电机：
      + 读取编码器里程:`print(left_driver.motor_angle)`
      + 写入电机速度(rad/s):`left_driver.velocity = 0.5`
    + 如果需要进行PID位置控制，首先修改类的PID参数变量，例如`left_driver.P = 0`，当然也可以使用默认值。然后启用并开始PID计算`left_driver.PID = True`,`left_driver.start_PID()`。然后就可以设置目标位置了`left_driver.target_angle`