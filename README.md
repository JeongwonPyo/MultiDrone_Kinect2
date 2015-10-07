# MultiDrone_Kinect2
-----------------------------------------------------
Copyright (C) 2015 Jeongwon Pyo
https://github.com/JeongwonPyo/MultiDrone_Kinect2
-----------------------------------------------------
Based on puku0k's CV Drone
https://github.com/puku0x/cvdrone
-----------------------------------------------------

How to use

  Base install & use are same as CV Drone.
  
  But this source must need kinect 2 and two AR.Drone 2.0. And visual studio 2013 is needed.
  
  Also, Need black belt on each hand. See this video : https://youtu.be/PqB-aZ20U1Y
  
  Kinect2 must be connected with client computer. 
  
  Kinect SDK 2.0 directory is basically C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\
  
  If directory is different, go to system variable and change the value of 'KINECTSDK20_DIR'. 
  
  Drones must be connected each computers, and each computers must be connected by ad-hoc network for TCP/IP communication.
  
  In the top of the Client's main.cpp, write server ip in IP variable.
  
  Don't need to touch any others.
  
  Run server project, after than run client project.
  
  After than, Hands will be found. 
  
  You will see some windows 'Left_Show_Thres', 'Right_Show_Thres', 'Left_Show_debug', and 'Right_Show_debug'.
  
  In the 'Left_Show_Thres', if you can see each fingers and 'Left_Show_debug' showing 5 rows(5 classes), press 't'.
  
  In the 'Right_Show_Thres', if you can see each fingers and 'Right_Show_debug' showing 5 rows(5 classes), press 'y'.
  
  So, New windows 'Left_FEMD_Base', and 'Right_FEMD_Base' will be shown.
  
  'Compare_FEMD_Left' and "Compare_FEMD_Right' are showing the result of compared classes.
  
  Finger gestures can see in this video : https://youtu.be/qIlGc0pJSyk
  

Finger gesture method based on the paper "Robust hand gesture recognition based on finger-earth mover's distance with a commodity depth camera".

But i made up simply.

This project is not a complete, and need more complement.

Just using this as reference. It is pretty unstable.

Thanks for reading.

