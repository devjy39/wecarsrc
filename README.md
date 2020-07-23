고명근선생님 gitlab : https://gitlab.com/gusto94/gunsan_edu

cmakeList 파일은 각 컴퓨터에 종속되므로 삭제

#tip
패키지 생성 cd catkin->src  =>   catkin_create_pkg 패키지이름 (std_msgs) rospy  => cm

권한 
		chmod +x f.py	sudo chmod 777 /dev/ttyACM0

코어 
		roslaunch vesc_driver vesc_driver_node.launch

테스트 
		rostopic pub /commands/motor/speed std_msgs/Float64 "data: 1700.0"


모라이시뮬레이터 켜는법
1.sudo simulator

2.rosbridge websocket

3.server연결 적용

4.센서연결 적용

5.rviz or rqt

모터
		rostopic pub /commands/motor/speed std_msgs/Float64 "data: 2000.0" -r 10

핸들
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



github 쓰는법
새폴더에서 git init 치면 깃 로컬 저장소가 된다

		git status 상태보기

		touch example1.py 파일생성
		git status

		git add .   추가
		git status

		git config --global user.email "wodufdl48@naver.com" 
		git config --global user.name "

		git commit -m "init" 커밋하기 

		git log 커밋 로그 보기

		gitk 관리모드

		git branch dev 브랜치만들기

		git branch -a 브랜치 all보기

		git checkout 'branch명' 로그인느낌

		git merge 'branch명' 병합

깃에 올리기
		git remot add orgin git@github.com/아디/파일명

		ssh-keygen -> cd .ssh -> gedit id_rsa.pub 
하면 key값나옴 github에 등록하면 ip(컴퓨터)인증 그 후 업로드





