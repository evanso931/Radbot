The radbot folder is the source file for the ROS code. It does not include the ROS packages and dependencies that were installed. Lots of the package source file were edited to work with the ROS hardware.

The complementary filter node is launch from the IMU driver.

Fitans code is the rur_navigation.cpp file 

Disclaimer: The radbot_controller.cpp that is copied form the NOX robot project and then modified to suit the hardwar setup. The also applies to alot of param files which also use ROS wiki sources for inspiration. 

The values in the param files have been modified during testing, built generally most of values are the same as james brutons because he used the same hardware.

The radbot_bringup launch file is heavily inspired by the nox robot project. But all the other luach files have been programed by me.

The make and xml files have also been programmed by myself. The commenting is the make file is from other proejects but can be ignored.