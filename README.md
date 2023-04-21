# navigation_frame 导航框架

```
开发人员：杨工

开发记录：
————2023年4月19日
1、导航框架第一个版本，ROS封装，添加日志系统
2、添加任务模块

————2023年4月20日
1、添加机器人模型，四轮底盘
2、添加遥控节点
3、添加arbotix仿真插件

————2023年4月22日
1、添加当前位置获取
```

**整体思路**

打算写一个新的导航通用框架，将各个功能模块连接起来

**目录结构**

```
.
├── arbotix_ros 仿真插件
├── atom 小车模型
├── navigation 导航系统
│   ├── include
│   │   ├── context 连接上下文
│   │   │   └── context.h
│   │   ├── logger 日志系统
│   │   │   └── logger.h
│   │   ├── mission 任务收发
│   │   │   └── mission.h
│   │   ├── task 任务
│   │   │   ├── path_task.h
│   │   │   └── task.h
│   │   └── utility_tool 通用工具
│   │       └── utility_tool.h
│   ├── package.xml
│   ├── src
│   │   ├── context
│   │   │   └── context.cpp
│   │   ├── logger
│   │   │   └── logger.cpp
│   │   ├── main.cpp
│   │   ├── mission
│   │   │   └── mission.cpp
│   │   └── task
│   │       ├── path_task.cpp
│   │       └── task.cpp
│   └── srv
│       └── path.srv
└── teleop_twist_keyboard_cpp 键盘控制程序
```

**完善思路**
1、任务收发使用ROS服务的形式
2、定位信息和传感器信息接收






