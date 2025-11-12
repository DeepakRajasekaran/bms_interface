/*
 * Name: AtcBms.hpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.2
 * Description: Header for ATC BMS CAN ROS1 node with safe destructor.
 */

#ifndef BMS_INTERFACE_ATC_BMS_HPP
#define BMS_INTERFACE_ATC_BMS_HPP

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>

class AtcBms
{
public:
    explicit AtcBms(ros::NodeHandle& nh);
    ~AtcBms();

    void spin();

private:
    bool initCan(const std::string& iface);
    bool sendWakeup();
    void readFrames();
    void parseFrame(const struct can_frame& frame);
    void publishBatteryMsg();

private:
    int m_canSocket{-1};
    ros::Publisher m_batteryPub;
    ros::Rate m_loopRate;

    float m_voltage;
    float m_current;
    float m_capacity;
    float m_fullCapacity;
    float m_soc;
    float m_temperature;
    uint16_t m_cycleCount;
    bool m_gotInfo;
};

#endif  // BMS_INTERFACE_ATC_BMS_HPP
