#tip
패키지 생성 cd catkin->src  =>   catkin_create_pkg 패키지이름 (std_msgs) rospy  => cm

cmakeList 파일은 각 컴퓨터에 종속되므로 삭제

권한 
		chmod +x f.py	sudo chmod 777 /dev/ttyACM0

코어 
		roslaunch vesc_driver vesc_driver_node.launch

테스트 
		rostopic pub /commands/motor/speed std_msgs/Float64 "data: 1700.0"


- 모라이시뮬레이터 켜는법
1. sudo simulator

2. rosbridge websocket

3. server연결 적용

4. 센서연결 적용

5. rviz or rqt <br>

- 모터 데이터 조회
		rostopic pub /commands/motor/speed std_msgs/Float64 "data: 2000.0" -r 10

- 핸들 데이터 조회
		rostopic pub /commands/servo/position std_msgs/Float64 "data: 0.5304" -r 10

steering 실제값
left  mid   right
0.15 0.5304 0.85

odom토픽으로 rpridar rviz에서 보이기
		rosrun tf static_transform_publisher 0 0 1 0 0 0 1 /base_link /laser 100

패키지다운 
		git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git

맵저장 
		rosrun map_server map_saver

맵 키고 위치인식

맵서버연결
		rosrun map_server map_server map.yaml

		imu오돔키고 라이다토픽 키고

		amcl키기
		roslaunch amcl amcl_diff.launch 

		rqt에서 global_localization 토픽 service caller로 call 하면 map에 particle뿌려짐

		rviz에서 posearray 토픽 확인
