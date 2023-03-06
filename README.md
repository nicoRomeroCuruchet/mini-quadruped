# Mini-Quadruped

This open-source project is designed for anyone interested in researching low-cost quadruped robotic platforms. It's built using the **Jetson Nano board** and the MCU **STM32F103C8T6**. By combining the equations for rigid body rotations with leg inverse kinematics, I've developed an algorithm that accurately positions the multi-joint legs based on the orientation of the torso. With the incorporation of a gyroscope and accelerometer, and by fusing both measures, I'm able to obtain real-time information about the body's orientation. Additionally, I'm using a webcam to collect more data, which enables me to track hand movements.

**Here's a really funny gif of my quadruped following my hand for you all!**

![Demo](https://media.giphy.com/media/VpOj6hN5GWJ0BFpOUy/giphy-downsized-large.gif)

# Jetno Nano 

Jetson Nano is a small, powerful, and low-cost computer developed by NVIDIA, designed for building AI applications and projects. It features a quad-core ARM Cortex-A57 CPU, a 128-core NVIDIA Maxwell GPU, and 4GB of LPDDR4 memory. The Jetson Nano also includes a variety of I/O ports, including USB 3.0, HDMI, Gigabit Ethernet, and GPIO, making it easy to interface with various devices and sensors.

The Jetson Nano runs on a Linux-based operating system and supports a variety of popular machine learning frameworks, including TensorFlow, PyTorch, and MXNet. It can be used for a wide range of AI applications, such as object detection, image classification, natural language processing, and robotics. Its compact size and low power consumption make it ideal for building portable and embedded AI systems.

A hand tracking solution that can detect and track the position and movement of hands in real-time video was designed using **MediaPipe**, which is an open-source framework developed by Google that provides a variety of pre-built, customizable machine learning models and processing pipelines for tasks such as image and video analysis, facial recognition, hand tracking, and more. MediaPipe also provides a set of C++ and Python libraries for developers to easily incorporate these models and pipelines into their own applications.


# STM32F103C8

The STM32F103C8 is a popular microcontroller from STMicroelectronics, based on the ARM Cortex-M3 core. It features 64 KB of Flash memory and 20 KB of SRAM, as well as a variety of on-chip peripherals, such as timers, SPI, I2C, USART, and ADC. The STM32F103C8 is commonly used in a wide range of embedded systems, including industrial control systems, motor control applications, and consumer electronics. Its high-performance, low-power consumption, and rich set of features make it a popular choice for developers looking to build complex and efficient systems. The STM32F103C8 is also supported by a variety of development tools and software, including the STM32CubeMX software tool, which allows developers to configure and generate code for the microcontroller.

![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/stm32f103.webp)

# How to deploy the code in the MCU?

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

The **src/Core/Inc/servo_configuration.h** contains the setup to control the servos of each leg:

- **The Left Front Leg was handle via timmer 1 channels 1, 2, 4**
- **The Left Rear Leg was handle via timmer 3 channels 1, 3, 4**
- **The Right Front Leg was handle via timmer 1 and 2. Timmer 2 channels 3, 4 and timmer 1 channel 3**
- **The Right Rear Leg was handle via timmer 2 and 3. Timmer 2 channel 1, 2 and timmer 3 channel 2**

 and the file in src/mini-quadruped.ioc cotains the setup of the microcontroller it, can be accessed via the STM32CubeIDE:
![alt text](https://github.com/nicoRomeroCuruchet/mini-quadruped/blob/main/img/Screen%20Shot%202022-04-01%20at%2010.30.43.png)

# MPU6050

The MPU6050 measures rotational and linear motion, and provides accurate data about the quadruped's orientation and movement. It communicates with the host device using I2C interface and can be easily integrated into a variety of microcontroller-based projects. In this project, I am using I2C1, which is located on the pins 6 and 7 of the MCU, and handle with the library probided by Bulanov Konstantin.

# Jeton and MCU conection

