#include "task/path_task.h"

using namespace Sakura::Logger;

namespace Nav
{
    PathTask::PathTask() // 构造函数
    {
        info("构造函数");

        record_path_service_ = nh_.advertiseService("record_path_num", &PathTask::recordPathServer, this);
        pub_path_service_ = nh_.advertiseService("pub_path_num", &PathTask::pubPathServer, this);

        path_line_pub_ = nh_.advertise<nav_msgs::Path>("/path_line", 1);
        path_num_pub_ = nh_.advertise<std_msgs::Int32>("/path_num", 1);
    }

    PathTask::~PathTask() // 析构函数
    {
        exit();

        info("析构函数");
    }

    bool PathTask::initial()
    {
        info("初始化");

        outFile_.clear();
        path_sum_ = 0.0;

        is_path_server_ = false;
        is_record_path_ = false;
        receive_once_ = true;
        global_sum_ = 0;
        global_path_size_ = 0;

        return true;
    }
    Result PathTask::update()
    {
        info("循环执行");
        return Result::SUCCEED;
    }
    bool PathTask::exit()
    {
        info("退出");
        return true;
    }

    bool PathTask::recordPathServer(navigation::path::Request &req,
                                    navigation::path::Response &res)
    {
        outFile_.close(); // 先关闭原有的

        int current_path_num = req.num;
        if (current_path_num >= 1 && current_path_num <= 1000)
        {
            is_record_path_ = true;

            std::string str1 = "/home/ubuntu/robot_csv/path_";
            std::string str2 = std::to_string(current_path_num);
            std::string str3 = ".csv";
            std::string str4 = str1 + str2 + str3;
            info("%s", str4);

            info("开始录制线路：", current_path_num);
            outFile_.open(str4, std::ios::app);
            if (!outFile_)
            {
                erro("打开文件失败！");
                return false;
            }
            info("请移动机器人录制线路");
            path_index_ = 0;
            receive_once_ = false;

            ++path_sum_;
            res.sum = path_sum_;
            info("current_path_num path_sum_ is [%d] [%d]", current_path_num, path_sum_);
            return true;
        }
        else if (current_path_num == 0)
        {
            info("暂停录制线路");
            outFile_.close();
            is_record_path_ = false;
            return true;
        }
        else
        {
            is_path_server_ = false;
            warn("path num is error, please try again !!!");
            return false;
        }
        return true;
    }

    bool PathTask::pubPathServer(navigation::path::Request &req,
                                 navigation::path::Response &res)
    {
        int global_path_num = req.num;
        if (global_path_num >= 1 && global_path_num <= 1000)
        {
            std::string str1 = "/home/ubuntu/robot_csv/path_";
            std::string str2 = std::to_string(global_path_num);
            std::string str3 = ".csv";
            std::string str4 = str1 + str2 + str3;
            info("%s ", str4);

            // 读取保存的路径点
            std::ifstream data(str4);
            if (!data)
            {
                info("打开文件失败！");
                return false;
            }
            else
            {
                info("发布线路：%d", global_path_num);

                std::string line;
                std::vector<std::string> line_data;
                while (getline(data, line))
                {
                    std::stringstream lineStream(line);
                    std::string cell;
                    while (std::getline(lineStream, cell, ','))
                    {
                        line_data.push_back(cell);
                    }
                }

                vec_points_.clear();
                for (int i = 0; i < line_data.size(); i += 3)
                {
                    double _x = std::atof(line_data[i + 0].c_str());
                    double _y = std::atof(line_data[i + 1].c_str());
                    double _yaw = std::atof(line_data[i + 2].c_str());
                    Points pre_waypoints = {_x, _y, _yaw}; // 放入结构体中
                    vec_points_.push_back(pre_waypoints);
                }
                global_path_size_ = vec_points_.size();
                info("global_path_size_ is %d", global_path_size_);
                for (int i = 0; i < global_path_size_; i++)
                {
                    info("x  is %f, y is %f, yaw is %f", vec_points_[i].x, vec_points_[i].y, vec_points_[i].yaw);
                }
            }

            pubPathLine();               // 发布线路点
            pubPathNum(global_path_num); // 发布线路编号

            ++global_sum_;
            res.sum = global_sum_;
            info("global_path_num global_sum_ [%d] [%d]", global_path_num, global_sum_);
            return true;
        }
        else if (global_path_num == 0)
        {
            info("暂停发布线路");
            return true;
        }
        else
        {
            warn("path num is error, please try again !!!");
            return false;
        }
        return true;
    }

    void PathTask::pubPathLine()
    {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        path.poses.clear();

        for (int i = 0; i < global_path_size_; i++)
        {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = vec_points_[i].x;
            pose_stamped.pose.position.y = vec_points_[i].y;
            pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(vec_points_[i].yaw);
            path.poses.push_back(pose_stamped);
        }
        path_line_pub_.publish(path); // 发布全局线路
    }

    void PathTask::pubPathNum(int index)
    {
        std_msgs::Int32 msg;
        msg.data = index;
        path_num_pub_.publish(msg);
    }
}
