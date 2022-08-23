# mini-quadruped

This is an open source project for anyone who wants to research low-cost quadruped robotic platforms. 
The equations to express rotations in combination with the inverse kinematics, allowed to 
derive an algorithm to find out the position of the multi-joint legs as a function of the torso’s orientation. 
Via the incorporation of a gyroscope and accelerometer, and fusion both measures, it is posible to obtain the orientation 
of the body in real time. Those values and the information collected via the webcam allowed the hand tracking.

   ![Demo](https://media.giphy.com/media/VpOj6hN5GWJ0BFpOUy/giphy-downsized-large.gif)

# STM32CubeIDE

The STM32CubeIDE is an advanced C/C++ development platform with peripheral configuration, code generation, code compilation,
and debug features for STM32 microcontrollers and microprocessors. Is available for free for different operating systems at 
the following link: 

      https://www.st.com/en/development-tools/stm32cubeide.html  

The **src** directory contains the code that should be executed with the STM32CubeIDE and deployed on the microcontroller stm32f103c8t6.
Run STM32CubeIDE, go to File -> Import, pull down the tab 'General', select 'Project from Folder or Archive' and click next. 
Finally, choose the path of **src** where you have cloned in your pc.

In src/Core/Src/ (source file) and src/Core/Inc/ (header file) in this directory there are three files:

- **main**               : contains the main loop of the program that performs quadruped stabilization.
- **inverse_kinematics** : the functions to determine the motion of a quadruped leg to reach a desired position
- **MPU6050**            : contains the driver to handle the gyroscope and accelerometer, and combine them. Also implements **Kalman's algorithm** to filtering the measurements observed over time. The author of this library is **Bulanov Konstantin** and you can check out his github repo with this link https://github.com/leech001/MPU6050 


To control the quadruped it was used the following microcontroller:

![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/stm32f103.webp)

The **src/Core/Inc/servo_configuration.h** contains the setup to control the servos of each leg:

- The Left Front Leg was handle via timmer 1 channels 1,2,4
- The Left Rear Leg was handle via timmer 3 channels 1,3,4
- The Right Front Leg was handle via timmer 1 and 2. Timmer 2 channels 3,4 and timmer 1 channel 3
- The Right Rear Leg was handle via timmer 2 and 3. Timmer 2 channel 1,2 and timmer 3 channel 2

 and the file in src/mini-quadruped.ioc cotains the setup of the microcontroller it, can be accessed via the STM32CubeIDE:
 
![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/Screen%20Shot%202022-04-01%20at%2010.30.43.png)
