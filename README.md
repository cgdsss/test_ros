# test_ros
### Assignment Requirements

http://note.youdao.com/noteshare?id=61f18c56246c34475a7ccdb88620934e
### Build
   ```bash
   cd ~/catkin_ws
   catkin_make --only-pkg-with-deps test_ros
   ```
### Test
- launch gazebo simulator

  https://gitee.com/cgdsss/kejia_sim
- launch this node

  ```roslaunch test_ros node.launch```
- launch rviz

  ```roslaunch test_ros rviz.launch```
- Call service request at terminal

  ```rosservice call /scan_server "flag: -1"```
  
  ```rosservice call /scan_server "flag: 0"```
  
  ```rosservice call /scan_server "flag: 15"```
  
  ```rosservice call /scan_server "flag: 30"```
  
- Change the value of parameter a and b in [yaml file](https://github.com/cgdsss/test_ros/blob/master/cfg/params.yaml).
