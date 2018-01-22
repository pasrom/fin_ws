start subscriber on kitti 00: 

	rosrun ORB_SLAM2 Monosub 5 3 29 -25 48 -12 0.55 0.50 1 5
	rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5
	rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.45 0.40 1 5 1 0 1
	rosrun ORB_SLAM2 Monosub 1 3 29 -25 48 -12 0.55 0.50 1 5 
	
with Gaussian counters: 

	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 1
	
with Gaussian counters and contour detecton

	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 1 1
	
with contour and without gaussian counters

	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 0 1
	
with Gaussian counters and ground point filtering, no contour detection and normal thresholding of 75 degrees:

	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 1 0 1 75
	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 0 0 0 75
	rosrun ORB_SLAM2 Monosub2 10 1 29 -25 48 -12 0.55 0.50 1 5 1 0 1
	
without Gaussian counters and with ground point filtering and no contour: 

	rosrun ORB_SLAM2 Monosub 10 1 29 -25 48 -12 0.55 0.50 1 5 0 0 1
	
start subscriber on rosbag: 

	rosrun ORB_SLAM2 Monosub2 30 5 2 -2 2 -2 0.55 0.50 1 5
	rosrun ORB_SLAM2 Monosub2 30 2 6 -6 6 -6 0.55 0.50 1 5
	
with camera location 

	rosrun ORB_SLAM2 Monosub2 20 1 10 -15 20 -10 0.55 0.50 1 5 1 1 1 75
	rosrun ORB_SLAM2 Monosub2 20 1 10 -15 20 -10 0.55 0.50 1 5 1 0 1 75
	rosrun ORB_SLAM2 Monosub2 20 1 10 -15 20 -10 0.55 0.50 1 5 0 0 1 75
	rosrun ORB_SLAM2 Monosub2 10 2 10 -15 20 -10 0.55 0.50 1 5 0 1 0
