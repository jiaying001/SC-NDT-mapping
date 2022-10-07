#include "ndt_mapping.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ndt_mapping");

  ros::NodeHandle nh, pnh("~");

  //ndt_mapping ndt;
  ndt_mapping *map = new ndt_mapping(nh, pnh);

  
  std::thread loop_thread(&ndt_mapping::loopClosureThread, map);
  std::thread visual_thread(&ndt_mapping::visualThread, map);

  map -> run();

  ros::spin();

  loop_thread.join();
  visual_thread.join();

  return 0;
};

