#include "kvaser_can_driver.h"

// Yaml read function
void operator>>(const YAML::Node &node, CANMsg &msg)
{
  msg.device_channel = (uint8_t)node["device_channel"].as<int>();
  msg.can_id = (uint16_t)node["can_id"].as<int>();
  msg.bypass_channel = (uint16_t)node["bypass_channel"].as<int>();
}

CANConfig loadYMLFile(std::string ymlpath)
{
  CANConfig config;
  try
  {
    YAML::Node node = YAML::LoadFile(ymlpath);
    CANMsg msg;
    for (uint16_t i = 0; i < (uint16_t)node.size(); i++)
    {
      node[i] >> msg;
      config.msgs.emplace_back(msg);
    }
  }
  catch (YAML::Exception &e)
  {
    std::cerr << e.what() << std::endl;
  }
  return config;
}

// KvaserCanDriver()
// Constructor
KvaserCanDriver::KvaserCanDriver()
    : nh_(""), pnh_("~")
{
  canStatus stat;

  // ROS
  kvaser_can_tx_sub_ = nh_.subscribe("/system/can_tx", 10, &KvaserCanDriver::cantxCallback, this);
  kvaser_can_rx_pub_ = nh_.advertise<ros_kvaser_can_driver::CANFrame>("/system/can_rx", 10);

  // parameters
  int bitrate = 0;
  pnh_.param("device_channel_num", device_channel_num_, 1);
  pnh_.param("can_bitrate", bitrate, 500000);
  pnh_.param("can_tx_config_file", can_tx_config_file_, std::string("can_tx_config.yaml"));
  pnh_.param("can_rx_config_file", can_rx_config_file_, std::string("can_rx_config.yaml"));

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
  canInitializeLibrary();

  int32_t dev_channels = 0;
  stat = canGetNumberOfChannels(&dev_channels);
  if (device_channel_num_ > dev_channels)
  {
    ROS_ERROR("CAN channel number error.");
    ros::shutdown();
  }

  // Get Parameters from YAML file
  can_tx_config_.msgs.clear();
  can_rx_config_.msgs.clear();
  can_tx_config_ = loadYMLFile(can_tx_config_file_);
  can_rx_config_ = loadYMLFile(can_rx_config_file_);

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
      ROS_INFO("Open %d channel", i);

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
      ROS_INFO("Config %d channel", i);

      // Start
      canBusOn(hnd);
      ROS_INFO("CAN BUS ON %d channel", i);

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
        ROS_ERROR("CAN channel %d : Device open error.", i);
        break;
      case kvaser_canlib_exception::CANLIB_CONFIG_FAIL:
        ROS_ERROR("CAN channel %d : Device setting error.", i);
        break;
      default:
        ROS_ERROR("CAN channel %d : Device unexpected error.", i);
        break;
      }
      ros::shutdown();
      return;
    }
    catch (...)
    {
      ROS_ERROR("CAN channel %d : unknown error.", i);
      ros::shutdown();
      return;
    }
  }
  for (int32_t i = 0; i < device_channel_num_; ++i)
  {
      // Make receive thread
      receive_thread_list_.emplace_back(std::thread([this, i]() { this->canReceiveThread(i); }));
      ROS_INFO("Start %d receive thread", i);
  }
} //KvaserCanDriver()

// ~KvaserCanDriver()
// Destructor
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
} //~KvaserCanDriver()

// cantxCallback()
// Callback function
void KvaserCanDriver::cantxCallback(const ros_kvaser_can_driver::CANFrame::ConstPtr &can_frame)
{
  const int32_t time_out = 10; //msec
  for (int32_t i = 0; i < (int32_t)can_tx_config_.msgs.size(); ++i)
  {
    if ((uint16_t)can_tx_config_.msgs[i].can_id == (uint16_t)can_frame->can_id &&
        (uint8_t)can_tx_config_.msgs[i].device_channel == (uint8_t)can_frame->device_channel)
    {
      uint8_t can_data[8];
      for (uint8_t j = 0; j < (uint8_t)can_frame->can_dlc; ++j){
        can_data[j] = can_frame->can_data[j];
      }
      // CAN transfer
      canStatus stat;
      {
        //boost::mutex::scoped_lock(kvaser_mtx_[can_tx_config_.msgs[i].channel]);
        stat = canWriteWait(kvaser_hnd_[can_tx_config_.msgs[i].device_channel], (long)can_tx_config_.msgs[i].can_id, can_data, (unsigned int)can_frame->can_dlc, canMSG_STD, (unsigned long)time_out);
      }

      if (stat != canOK)
      {
        ROS_WARN("Cannot send CAN data ID : %d", (int32_t)can_tx_config_.msgs[i].can_id);
      }
      return;
    }
  }
}

// canReceiveThread()
// Receive thread function
void KvaserCanDriver::canReceiveThread(uint8_t channel_num)
{
  while (ros::ok())
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
        //boost::mutex::scoped_lock(kvaser_mtx_[channel_num]);
        res = canReadWait(kvaser_hnd_[channel_num], &id, &rxmsg, &dlc, &flag, &kv_time_stamp, timeout_ms);
        //res = canRead(kvaser_hnd_[channel_num], &id, &rxmsg, &dlc, &flag, &kv_time_stamp);
      }
      if (res != canOK)
      {
        ROS_WARN_DELAYED_THROTTLE(10,"CAN Error");
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
        //ROS_INFO("CAN Receive %d 0x%3X",channel_num,(int)id);
            ros_kvaser_can_driver::CANFrame frame;
            frame.device_channel = channel_num;
            frame.kvaser_stamp = kv_time_stamp;
            frame.can_id = id;
            frame.can_dlc = dlc;
            for (int8_t j = 0; j < (int8_t)dlc; ++j)
            {
              frame.can_data.emplace_back(rxmsg[j]);
            }
            kvaser_can_rx_pub_.publish(frame);

            if(can_rx_config_.msgs[i].bypass_channel < device_channel_num_ && can_rx_config_.msgs[i].bypass_channel != channel_num){
              // CAN transfer
              canStatus stat;
              {
                //boost::mutex::scoped_lock(kvaser_mtx_[can_tx_config_.msgs[i].channel]);
                stat = canWriteWait(kvaser_hnd_[can_rx_config_.msgs[i].bypass_channel], (long)id, rxmsg, (unsigned int)dlc, canMSG_STD, timeout_ms);
              }

              if (stat != canOK)
              {
                ROS_WARN_DELAYED_THROTTLE(10,"Cannot send CAN data ID : %d", (int32_t)can_tx_config_.msgs[i].can_id);
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
        ROS_WARN_DELAYED_THROTTLE(10,"CAN ch%d: Receive no data.", channel_num);
        //usleep(100);
        continue;
      case kvaser_canlib_exception::CANLIB_RECV_ERR:
        ROS_WARN_DELAYED_THROTTLE(10,"CAN ch%d: Receive Error.", channel_num);
        break;
      default:
        ros::shutdown();
        break;
      }
    }
    catch (...)
    {
      ROS_ERROR("CAN ch%d: Unexpected Error.", channel_num);
      ros::shutdown();
    }
  }
}

int32_t main(int32_t argc, char **argv)
{
  // Setup ROS.
  ros::init(argc, argv, "ros_kvaser_can_driver_node");
  KvaserCanDriver node;

  ros::spin();
  std::cerr << "\nros_kvaser_can_driver_node: Exiting...\n";
  return (EXIT_SUCCESS);
} // end main()
