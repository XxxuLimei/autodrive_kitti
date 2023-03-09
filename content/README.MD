1. 发布kitti的camera_02（左侧彩色相机）视频  
2. 发布kitti的点云视频(velodyne)  
3. 画出中心的车模型，并画出相机的视野  
4. 画出车的imu检测数据，包括线速度，角速度和加速度；同时添加了无法进行可视化的gps数据，可以在终端使用`rostopic echo /kitti_gps`命令查看  
5. 在Jupyter Notebook上写出在单帧图片上画出检测框的程序  
6. 在RViz的相机视角发布2D检测框  
7. 在Jupyter Notebook上写出在单帧点云上画出3D检测框的程序，包含了如何从相机2坐标系转换到点云坐标系  
8. 在RViz的点云视角发布3D检测框  
9. 将track_id发布到camera和velodyne上，通过rviz显示  
10. 在Jupyter Notebook上画出自己的轨迹  
11. 将自己的轨迹发布到RViz上  
12. 将自己的轨迹和相机视野内所有物体的轨迹发布到RViz上  
