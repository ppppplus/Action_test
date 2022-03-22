# Action_test

### 关键文件说明

- scripts
  - elevator_server.py 进出电梯的ros service服务端
  - inside/outside_nav_server.py 室内/外导航的ros service服务端
  - hive_box_smach.py 利用smach_ros构建状态机，进行配送任务状态流转的脚本

- src
  - box_approach_server.cpp 向机柜靠近的ros action服务端
  - box_focous_server.cpp 与机柜进行细节对准的ros action服务端
