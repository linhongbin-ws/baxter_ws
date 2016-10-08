This package can be used to run the pick n place demo that has been demonstrated at the 2016 ROS Training at GDUT.

For using with the real baxter robot run the following in different terminals:
1. roslaunch openni2_launch openni2.launch         # starts necessary programs to get data from an Asus Xtion Pro/Kinect.
2. rosrun pa_localization visual_localization2.py  # starts our visual localiztion program on a flat surface (requires you to have calibrated first).
3. roslaunch pa_demo pa_1.launch                   # This will start the pick and place task

---------------
For calibration you need to run:
rosrun pa_localization table_pos_calibration.py

The readme file in that demo will explain calibration.
