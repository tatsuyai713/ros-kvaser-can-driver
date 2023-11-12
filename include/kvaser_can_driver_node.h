#ifndef _KVASER_CAN_DRIVER_H_
#define _KVASER_CAN_DRIVER_H_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>
#include <fstream>
#include <sstream>

#include <canlib.h>
#include "yaml-cpp/yaml.h"

#include "ros_kvaser_can_driver/msg/can_frame.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp" // 必要なヘッダーのインクルード


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


class KvaserCanDriver : public rclcpp::Node {
public:
    KvaserCanDriver();
    ~KvaserCanDriver();

private:
  int device_channel_num_;
  int can_bitrate_;

  CANConfig can_tx_config_;
  CANConfig can_rx_config_;
  std::string can_tx_config_path_;
  std::string can_rx_config_path_;

  // Kvaser Canlib
  std::vector<std::thread> receive_thread_list_;
  std::vector<CanHandle> kvaser_hnd_;
  std::mutex *kvaser_mtx_;
    void start_main_loop();
    void publish_custom_message();

  // callback functions
  void cantxCallback(const ros_kvaser_can_driver::msg::CANFrame::SharedPtr msg);

  // thread
  void canReceiveThread(uint8_t channel_num);

  // Subscriber
    rclcpp::Subscription<ros_kvaser_can_driver::msg::CANFrame>::SharedPtr kvaser_can_tx_sub_;
    rclcpp::Publisher<ros_kvaser_can_driver::msg::CANFrame>::SharedPtr kvaser_can_rx_pub_; 
};

#endif  // _KVASER_CAN_DRIVER_H_