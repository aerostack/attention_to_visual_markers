#include "../include/qr_code_localizer.h"
#include <boost/thread/thread.hpp>


int main(int argc, char** argv){
  ros::init(argc, argv, ros::this_node::getName());
  std::cout << ros::this_node::getName() << std::endl;

  QrCodeLocalizer detector;
  detector.ownSetUp();
  ros::spin();
  return 0;
}
