The different file to convert data from elios3 to SVO (VINS) usable data.

Step 1 : 
1) Create a pointcloud fro google map (sample uniformely obj). Be carefull to export with Axe y forward, z up from blender.
2) Optionnal : convert the lidar data to ENU
3) Make ground truth out of COLMAP : need to find scale + transform.
4) Add colmap to bag
5) Read gpx file
6) Align GPS stamp
7) Optionnal : convert to pinhole camera for VINS.
