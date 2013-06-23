  guidebot Navigation, spring 2012
  Copyright by Dung Le, David Glover, Tochukwu Nwaoduh 2012
  =============================================================================
  Tested platform: Linux + mrpt lib, guidebbot (ActivMediaRobot) base + Kinect 
  For initial MRPT library and Kinect setup please see our project report.  
  additional files: guidebotNavConf.ini, floorplan (map), run script(optional)
  =============================================================================
  implementing:
  - main thread: command prompt for 
    + robot status
    + manual drive
    + path planning and driving robot to target
  - display thread: 3D display of current robot movement, also shows sensor 
    readings(kinect and sonar) and particles distribution 
  - pdf update thread: execute montecarlo localization using particles filter
    based on the a current pair of action-observation.  
  - kinect observation grabbing thread
  - wall detect thread
  =============================================================================
  Current status and issues 
  - path planning and 3d display works
  - driving is smooth, however does not work well with wall detect.
  - localization works in principle however needed some tuning. Particles are 
    calculated in background and updated to our display ONLY. We have NOT update
    current robot location (robot odometry, where robot thinks it is at) with the
    most likely location (calculated from particle filter). We think there is an 
    issue with the changeOdometry() function from MRPT. A temporary fix for this
    (odometryOffset) is implemented, but not thoroughly tested.