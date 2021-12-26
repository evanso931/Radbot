Potentially to open the Simulink model you may need the following plugins, an image of the final block diagram is found in my report:

https://uk.mathworks.com/videos/how-to-automatically-tune-pid-controllers-1600850427273.html Video on how to use closed loop PID Autotuner, was followed to bases of design 

https://www.mathworks.com/help/slcontrol/ug/closedlooppidautotuner.html Closed loop auto tune block Simulink 

https://www.mathworks.com/videos/pid-controller-design-for-a-dc-motor-68881.html Plant could not be linearized so couldn’t use this method. Also system is no linear and has complex behaviour so couldn’t simulate and a plant isn’t available for the motor. 

https://www.mathworks.com/discovery/pid-control.html What is PID Control MATlab

https://www.mathworks.com/matlabcentral/fileexchange/39354-device-drivers Used Device Drivers add on blocks to be able to read the encoder counts in Simulink other wise would have to create my own Simulink function block from my already working matlab encoder code.

https://www.mathworks.com/help/mcb/ref/quadraturedecoder.html Used quadrature decoder to read the speed of the motor as PID for just using encoder counts per second didn’t work. This was from the Motor Control Block set add on.
