# mini-quadruped

This open source project is designed for anyone interested in researching low-cost quadruped robotic platforms, and it's built on the Jetson Nano board and blue pill (STM32F103C8T6). By combining equations for rotations with inverse kinematics, we've derived an algorithm that accurately positions the multi-joint legs based on the orientation of the torso. With the incorporation of a gyroscope and accelerometer, and by fusing both measures, we're able to obtain real-time information about the body's orientation. Additionally, we use a webcam to collect more data, which enables us to track hand movements.

   ![Demo](https://media.giphy.com/media/VpOj6hN5GWJ0BFpOUy/giphy-downsized-large.gif)

# STM32CubeIDE

The STM32CubeIDE is an advanced C/C++ development platform with peripheral configuration, code generation, code compilation,
and debug features for STM32 microcontrollers and microprocessors. Is available for free for different operating systems at 
the following link: 

      https://www.st.com/en/development-tools/stm32cubeide.html  

The **src** directory contains the code that should be executed with the STM32CubeIDE and deployed on the blue pill (STM32F103C8T6).
Run STM32CubeIDE, go to File -> Import, pull down the tab 'General', select 'Project from Folder or Archive' and click next. 
Finally, choose the path of **src** where you have cloned in your pc.

In src/Core/Src/ (source file) and src/Core/Inc/ (header file) in this directory there are three files:

- **main**               : contains the main loop of the program that performs quadruped stabilization.
- **inverse_kinematics** : the functions to determine the motion of a quadruped leg to reach a desired position
- **MPU6050**            : contains the driver to handle the gyroscope and accelerometer, and combine them. Also implements **Kalman's algorithm** to filtering the measurements observed over time. The author of this library is **Bulanov Konstantin** and you can check out his github repo with this link https://github.com/leech001/MPU6050 


To control the servo motors and the mpu6050 of the quadruped it was used the following microcontroller:

![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/stm32f103.webp)

The **src/Core/Inc/servo_configuration.h** contains the setup to control the servos of each leg:

- The Left Front Leg was handle via timmer 1 channels 1,2,4
- The Left Rear Leg was handle via timmer 3 channels 1,3,4
- The Right Front Leg was handle via timmer 1 and 2. Timmer 2 channels 3,4 and timmer 1 channel 3
- The Right Rear Leg was handle via timmer 2 and 3. Timmer 2 channel 1,2 and timmer 3 channel 2

 and the file in src/mini-quadruped.ioc cotains the setup of the microcontroller it, can be accessed via the STM32CubeIDE:
 
![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/Screen%20Shot%202022-04-01%20at%2010.30.43.png)

# MPU6050

The MPU6050 is a popular integrated circuit (IC) that combines a three-axis gyroscope and a three-axis accelerometer into a single package. It is commonly used as a motion tracking sensor in various applications, such as drones, robotics, and gaming controllers. The MPU6050 measures rotational and linear motion, and provides accurate data about the device's orientation and movement. It communicates with the host device using I2C interface and can be easily integrated into a variety of microcontroller-based projects.
