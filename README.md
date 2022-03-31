# mini-quadruped

This is an open source project for anyone who wants to research low-cost quadruped robotic platforms. 
The equations to express rotations in combination with the inverse kinematics, allowed to 
derive an algorithm to find out the position of the multi-joint legs as a function of the torso’s orientation. 
Via the incorporation of a gyroscope and accelerometer, and merging both measures, is posible to obtain the orientation 
of the body in real time. Those values and the information collected via the webcam allowed to the robot to 
follow the position of the hand in real time.

   ![Demo](https://media.giphy.com/media/i7qEreajtPtJEAiz5Z/giphy-downsized-large.gif)

# STM32CubeIDE

The **src** directory contains the code that should be executed with the STM32CubeIDE and deployed on the microcontroller stm32f103c8t6. 
The STM32CubeIDE is an advanced C/C++ development platform with peripheral configuration, code generation, code compilation, and debug features for STM32 microcontrollers and microprocessors. Is available for free for different operating systems at the following link: 

                           https://www.st.com/en/development-tools/stm32cubeide.html  

Run STM32CubeIDE, go to File -> Import, pull down the tab 'General', select 'Project from Folder or Archive' and click next. 
Finally choose the path the src where you have cloned this directory on your pc
