
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/package.h>
#include <vector>
#include <string>
#include <thread>
#include <mutex>

#include <boost/thread.hpp>

#include <canlib.h>
#include <yaml-cpp/yaml.h>
#include <ros_kvaser_can_driver/CANFrame.h>

enum class kvaser_canlib_exception
{
  CANLIB_OPEN_FAIL,
  CANLIB_CONFIG_FAIL,
  CANLIB_RECV_ERR,
  CANLIB_NO_MSG,
  CANLIB_TIMEOUT,
};

typedef struct CANMsg
{
  uint8_t device_channel;
  uint16_t can_id;
  uint8_t bypass_channel;
} CANMsg;

typedef struct CANConfig
{
  std::vector<CANMsg> msgs;
} CANConfig;

class KvaserCanDriver
{
private:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // Publisher
  ros::Publisher kvaser_can_rx_pub_;
  // Subscriber
  ros::Subscriber kvaser_can_tx_sub_;

  int device_channel_num_;
  int can_bitrate_;

  std::string can_tx_config_file_;
  std::string can_rx_config_file_;

  CANConfig can_tx_config_;
  CANConfig can_rx_config_;

  // Kvaser Canlib
  std::vector<std::thread> receive_thread_list_;
  std::vector<CanHandle> kvaser_hnd_;
  std::mutex *kvaser_mtx_;

  // callback functions
  void cantxCallback(const ros_kvaser_can_driver::CANFrame::ConstPtr &can_frame);

  // thread
  void canReceiveThread(uint8_t channel_num);

public:
  // Constructor.
  KvaserCanDriver();

  // Destructor.
  ~KvaserCanDriver();
};
