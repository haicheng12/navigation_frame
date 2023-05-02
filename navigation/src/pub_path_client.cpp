#include "ros/ros.h"
#include "navigation/path.h"
#include <cstdlib>
#include "message/message.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub_path_client");
  if (argc != 2)
  {
    ROS_INFO("usage: pub_path_client X");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<navigation::path>("pub_path_num");

  navigation::path srv;
  srv.request.num = atoll(argv[1]);
  if (client.call(srv))
  {
    ROS_INFO("pub path sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call pub_path_client");
    return 1;
  }

  return 0;
}
