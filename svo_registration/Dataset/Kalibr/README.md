# First step : calibration :

1)convert to kalibr readable bag 
2) run kalibr with : 
rosrun kalibr kalibr_calibrate_cameras --target /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Kalibr/april_6x6.yaml --models pinhole-equi --topics /camera_2/image_raw --bag /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Calib.bag --show-extraction 

rosrun kalibr kalibr_calibrate_imu_camera --bag /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Calib.bag --cam /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Kalibr/Calib-camchain.yaml --imu /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Kalibr/imu.yaml --target /home/roxane/elios3_2706/EL300825315585_01644_213_2hand/Kalibr/april_6x6.yaml 
