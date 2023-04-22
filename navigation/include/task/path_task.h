#ifndef _PATH_TASK_H_
#define _PATH_TASK_H_

#include "task.h"
#include <iostream>
#include <fstream>
#include <vector>
#include "math.h"
#include <string>
#include "logger/logger.h"
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include "navigation/path.h"

enum PathTaskState : u_int8_t // 录制线路状态
{
	RUNNING = 1, // 正在录制
	STOP,				 // 停止录制
	PUB					 // 发布线路
};

struct Points // 全局路径
{
	double x;
	double y;
	double yaw;
};

namespace Nav
{
	// 派生类
	class PathTask : public Task
	{
	public:
		PathTask();
		virtual ~PathTask();

		bool initial() override;
		Result update() override;
		bool exit() override;

	private:
		ros::NodeHandle nh_;

		ros::ServiceServer record_path_service_;
		ros::ServiceServer pub_path_service_;

		ros::Publisher path_line_pub_;
		ros::Publisher path_num_pub_;

		bool recordPathServer(navigation::path::Request &req,
													navigation::path::Response &res); // 记录路径
		bool pubPathServer(navigation::path::Request &req,
											 navigation::path::Response &res); // 发布路径

		void pubPathLine();					// 发布路径
		void pubPathNum(int index); // 发布路径标号

		std::ofstream outFile_;					 // 保存文件
		std::vector<Points> vec_points_; // 路径点存储
		bool is_record_path_;
		bool is_path_server_;
		bool receive_once_;
		int path_index_;
		int path_sum_;
		int global_path_size_;
		int global_sum_;
	};
}

#endif