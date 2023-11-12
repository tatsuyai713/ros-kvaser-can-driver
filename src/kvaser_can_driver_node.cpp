#include "kvaser_can_driver_node.h"

KvaserCanDriver::KvaserCanDriver() : Node("data_analysis_node")
{
  // パラメータの取得
  this->declare_parameter("device_channel_num", 1);
  this->get_parameter("device_channel_num", device_channel_num_);
  RCLCPP_INFO(this->get_logger(), "device_channel_num : %d", device_channel_num_ );
  int bitrate = 0;
  this->declare_parameter("can_bitrate", 500000);
  this->get_parameter("can_bitrate", bitrate);
  RCLCPP_INFO(this->get_logger(), "bitrate : %d", bitrate );

  this->declare_parameter("can_tx_config_path", "can_tx_config.yaml");
  this->get_parameter("can_tx_config_path", can_tx_config_path_);
  this->declare_parameter("can_rx_config_path", "can_rx_config.yaml");
  this->get_parameter("can_rx_config_path", can_rx_config_path_);
  std::string package_path = ament_index_cpp::get_package_share_directory("ros_kvaser_can_driver");
  can_tx_config_path_ = package_path + "/config/" + can_tx_config_path_;
  can_rx_config_path_ = package_path + "/config/" + can_rx_config_path_;

  // ファイルからYAMLデータを読み込む
  std::ifstream can_tx_config_fin(can_tx_config_path_); // ファイルのパスを指定
  std::ifstream can_rx_config_fin(can_rx_config_path_); // ファイルのパスを指定
  // ファイルが開けたかを確認
  if (!can_tx_config_fin)
  {
    RCLCPP_ERROR(this->get_logger(), "File not found or unable to open" );
    rclcpp::shutdown();
    return;
  }
  if (!can_rx_config_fin)
  {
    RCLCPP_ERROR(this->get_logger(), "File not found or unable to open" );
    rclcpp::shutdown();
    return;
  }

  std::stringstream can_tx_config_buffer;
  can_tx_config_buffer << can_tx_config_fin.rdbuf();
  std::string can_tx_config_yaml_data = can_tx_config_buffer.str();
  YAML::Node can_tx_config_yaml = YAML::Load(can_tx_config_yaml_data);
  for (auto const &msg : can_tx_config_yaml["can_tx_config"])
  {
    CANMsg can_msg;
    can_msg.device_channel = msg["device_channel"].as<uint8_t>();
    can_msg.can_id = msg["can_id"].as<uint16_t>();
    can_msg.bypass_channel = -1;
    can_tx_config_.msgs.push_back(can_msg);
  }

  std::stringstream can_rx_config_buffer;
  can_rx_config_buffer << can_rx_config_fin.rdbuf();
  std::string can_rx_config_yaml_data = can_rx_config_buffer.str();
  YAML::Node can_rx_config_yaml = YAML::Load(can_rx_config_yaml_data);
  for (auto const &msg : can_rx_config_yaml["can_rx_config"])
  {
    CANMsg can_msg;
    can_msg.device_channel = msg["device_channel"].as<uint8_t>();
    can_msg.can_id = msg["can_id"].as<uint16_t>();
    can_msg.bypass_channel = msg["bypass_channel"].as<uint8_t>();
    can_rx_config_.msgs.push_back(can_msg);
  }

  // Existing subscription logic
  kvaser_can_tx_sub_ = this->create_subscription<ros_kvaser_can_driver::msg::CANFrame>(
      "data_topic", 10, std::bind(&KvaserCanDriver::cantxCallback, this, std::placeholders::_1));

  // Publisher creation logic for custom message
  kvaser_can_rx_pub_ = this->create_publisher<ros_kvaser_can_driver::msg::CANFrame>("/system/can_rx", 10);

  switch (bitrate)
  {
  case 1000000:
    can_bitrate_ = canBITRATE_1M;
    break;
  case 500000:
    can_bitrate_ = canBITRATE_500K;
    break;
  case 250000:
    can_bitrate_ = canBITRATE_250K;
    break;
  case 125000:
    can_bitrate_ = canBITRATE_125K;
    break;
  default:
    can_bitrate_ = canBITRATE_500K;
    break;
  }

  // Kvaser canlib initialize
  canStatus stat;
  int32_t dev_channels = 0;
  canInitializeLibrary();
  stat = canGetNumberOfChannels(&dev_channels);
  if (device_channel_num_ > dev_channels)
  {
    RCLCPP_ERROR(this->get_logger(), "CAN channel number error.");
    rclcpp::shutdown();
    return;
  }

  receive_thread_list_.clear();
  kvaser_mtx_ = new std::mutex[device_channel_num_];

  for (int32_t i = 0; i < device_channel_num_; ++i)
  {
    try
    {
      // Open
      CanHandle hnd;
      hnd = canOpenChannel(i, canOPEN_EXCLUSIVE);
      if (hnd < 0)
      {
        throw(kvaser_canlib_exception::CANLIB_OPEN_FAIL);
      }
      RCLCPP_INFO(this->get_logger(), "Open %d channel", i);

      // Setting
      stat = canSetBusParams(hnd, can_bitrate_, 0, 0, 0, 0, 0);
      if (stat != canOK)
      {
        throw(kvaser_canlib_exception::CANLIB_CONFIG_FAIL);
      }

      stat = canSetBusOutputControl(hnd, canDRIVER_NORMAL);
      if (stat != canOK)
      {
        throw(kvaser_canlib_exception::CANLIB_CONFIG_FAIL);
      }
      RCLCPP_INFO(this->get_logger(), "Config %d channel", i);

      // Start
      canBusOn(hnd);
      RCLCPP_INFO(this->get_logger(), "CAN BUS ON %d channel", i);

      // Clear buffer
      canFlushReceiveQueue(hnd);

      // Save handle
      kvaser_hnd_.emplace_back(hnd);
    }
    catch (const kvaser_canlib_exception &ex)
    {
      switch (ex)
      {
      case kvaser_canlib_exception::CANLIB_OPEN_FAIL:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device open error.", i);
        break;
      case kvaser_canlib_exception::CANLIB_CONFIG_FAIL:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device setting error.", i);
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "CAN channel %d : Device unexpected error.", i);
        break;
      }
      rclcpp::shutdown();
      return;
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "CAN channel %d : unknown error.", i);
      rclcpp::shutdown();
      return;
    }
  }
  for (int32_t i = 0; i < device_channel_num_; ++i)
  {
    // Make receive thread
    receive_thread_list_.emplace_back(std::thread([this, i]()
                                                  { this->canReceiveThread(i); }));
    RCLCPP_INFO(this->get_logger(), "Start %d receive thread", i);
  }
}

KvaserCanDriver::~KvaserCanDriver()
{
  for (int32_t i = 0; i < device_channel_num_; ++i)
  {
    if ((int32_t)receive_thread_list_.size() > i)
    {
      receive_thread_list_[i].join();
      canBusOff(kvaser_hnd_[i]);
      canClose(kvaser_hnd_[i]);
    }
  }
  delete kvaser_mtx_;
}

// Callback function
void KvaserCanDriver::cantxCallback(const ros_kvaser_can_driver::msg::CANFrame::SharedPtr can_frame)
{
  const int32_t time_out = 50; // msec
  for (int32_t i = 0; i < (int32_t)can_tx_config_.msgs.size(); ++i)
  {
    if ((uint16_t)can_tx_config_.msgs[i].can_id == (uint16_t)can_frame->can_id &&
        (uint8_t)can_tx_config_.msgs[i].device_channel == (uint8_t)can_frame->device_channel)
    {
      uint8_t can_data[8];
      for (uint8_t j = 0; j < (uint8_t)can_frame->can_dlc; ++j)
      {
        can_data[j] = can_frame->can_data[j];
      }
      // CAN transfer
      canStatus stat;
      {
        // boost::mutex::scoped_lock(kvaser_mtx_[can_tx_config_.msgs[i].channel]);
        stat = canWriteWait(kvaser_hnd_[can_tx_config_.msgs[i].device_channel], (long)can_tx_config_.msgs[i].can_id, can_data, (unsigned int)can_frame->can_dlc, canMSG_STD, (unsigned long)time_out);
      }

      if (stat != canOK)
      {
        RCLCPP_WARN(this->get_logger(), "Cannot send CAN data ID : %d", (int32_t)can_tx_config_.msgs[i].can_id);
      }
      return;
    }
  }
}

// canReceiveThread()
// Receive thread function
void KvaserCanDriver::canReceiveThread(uint8_t channel_num)
{
  rcutils_duration_value_t throttle_period = 10000; // スロットリングの時間間隔 (ここでは10秒)

  while (rclcpp::ok())
  {
    long id = 0;
    unsigned int dlc = 0;
    uint8_t rxmsg[8];
    unsigned long kv_time_stamp = 0;
    unsigned int flag = 0;
    const unsigned long timeout_ms = 10;

    // Read CAN Data
    try
    {
      int res;
      {
        // boost::mutex::scoped_lock(kvaser_mtx_[channel_num]);
        res = canReadWait(kvaser_hnd_[channel_num], &id, &rxmsg, &dlc, &flag, &kv_time_stamp, timeout_ms);
        // res = canRead(kvaser_hnd_[channel_num], &id, &rxmsg, &dlc, &flag, &kv_time_stamp);
      }
      if (res != canOK)
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "CAN Error");
        if (res == canERR_NOMSG)
        {
          throw(kvaser_canlib_exception::CANLIB_NO_MSG);
        }
        else if (res == canERR_TIMEOUT)
        {
          throw(kvaser_canlib_exception::CANLIB_TIMEOUT);
        }
        else
        {
          throw(kvaser_canlib_exception::CANLIB_RECV_ERR);
        }
      }
      else
      {
        // Processing data
        for (int i = 0; i < (int)can_rx_config_.msgs.size(); ++i)
        {
          if (can_rx_config_.msgs[i].can_id == id &&
              can_rx_config_.msgs[i].device_channel == channel_num)
          {
            // RCLCPP_INFO(this->get_logger(), "CAN Receive %d 0x%3X",channel_num,(int)id);
            auto frame = ros_kvaser_can_driver::msg::CANFrame();
            frame.device_channel = channel_num;
            frame.kvaser_stamp = kv_time_stamp;
            frame.can_id = id;
            frame.can_dlc = dlc;
            for (int8_t j = 0; j < (int8_t)dlc; ++j)
            {
              frame.can_data.emplace_back(rxmsg[j]);
            }
            kvaser_can_rx_pub_->publish(frame);

            if (can_rx_config_.msgs[i].bypass_channel < device_channel_num_ && can_rx_config_.msgs[i].bypass_channel != channel_num)
            {
              // CAN transfer
              canStatus stat;
              {
                // boost::mutex::scoped_lock(kvaser_mtx_[can_tx_config_.msgs[i].channel]);
                stat = canWriteWait(kvaser_hnd_[can_rx_config_.msgs[i].bypass_channel], (long)id, rxmsg, (unsigned int)dlc, canMSG_STD, timeout_ms);
              }

              if (stat != canOK)
              {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "Cannot send CAN data ID : %d", (int32_t)can_tx_config_.msgs[i].can_id);
              }
            }
          }
        }
      }
    }
    catch (const kvaser_canlib_exception &ex)
    {
      switch (ex)
      {
      case kvaser_canlib_exception::CANLIB_NO_MSG:  // no data in the kvaser buffer
      case kvaser_canlib_exception::CANLIB_TIMEOUT: // no data in the kvaser buffer
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "CAN ch%d: Receive no data.", channel_num);
        // usleep(100);
        continue;
      case kvaser_canlib_exception::CANLIB_RECV_ERR:
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "CAN ch%d: Receive error.", channel_num);
        break;
      default:
        rclcpp::shutdown();
        break;
      }
    }
    catch (...)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this, throttle_period, "CAN ch%d: Unexpected error.", channel_num);
      rclcpp::shutdown();
    }
  }
}
