
- 구동 환경 
	
	1. Ubuntu 16.04 or 18.04

	2. ROS

	3. OpenCV (ROS 설치 시 자동 설치)

- 실행 방법


	1. A*

	optimization_class_ws 폴더의 경로에서 터미널 창에 다음 명령어 입력

		1. catkin_make
	
		2. source devel/setup.bash

		3. rosrun a_star_algorithm_pkg a_star_algorithm_pkg [img_num] [init_x] [init_y] [final_x] [final_y]

		   ex. rosrun a_star_algorithm_pkg a_star_algorithm_pkg 5 1 1 920 500

		- 명령어 변수 설명

			1. img_num은 실행할 Obstacle Images로 1~7의 숫자 입력 가능

			2. init_xy는 시작 위치 (x는 3~937, y는 3~539)

			3. final_xy는 목표 위치 (x는 3~937, y는 3~539)




	2. MGPF-Hybrid A*

	optimization_class_ws 폴더의 경로에서 터미널 창에 다음 명령어 입력

		1. catkin_make

		2. source devel/setup.bash

		3. rosrun hybrid_a_star_algorithm_pkg hybrid_a_star_algorithm_pkg_exe




 -----------------------------------------------------

 - Version

	1. 2020.03.16 A* 알고리즘 추가

	2. 2020.06.05 Hybrid A* 알고리즘 추가

	3. 2020.08.26 Hybrid A*를 MGPF-Hybrid A* (Multi-Goals Potential Field Hybrid A*)로 교체


