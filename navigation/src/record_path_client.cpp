#include "ros/ros.h"
#include "navigation/path.h"
#include <cstdlib>
#include "message/message.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "record_client");
  if (argc != 2)
  {
    ROS_INFO("usage: record_num X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<navigation::path>("record_path_num");

  navigation::path srv;
  srv.request.num = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("record sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call record_num");
    return 1;
  }

  return 0;
}