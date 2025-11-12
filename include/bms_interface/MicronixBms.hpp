/*
 * Name: MicronixBms.hpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.0
 * Description: Header file for Micronix BMS CAN ROS1 node.
 */

#ifndef MICRONIX_BMS_HPP
#define MICRONIX_BMS_HPP

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <cstdint>

class MicronixBms
{
public:
    explicit MicronixBms(ros::NodeHandle& nh);
    ~MicronixBms();

    void spin();

private:
    bool initCan(const std::string& iface);
    bool sendRequest(uint32_t id);
    void readFrames();
    void publishBatteryMsg();

    void parse0x100(const uint8_t* d);
    void parse0x101(const uint8_t* d);
    void parse0x103(const uint8_t* d);

    uint16_t calculateCrc16(const uint8_t* data, size_t len);

    int m_canSocket;
    ros::Publisher m_batteryPub;
    ros::Rate m_loopRate;

    float m_voltage;
    float m_current;
    float m_capacity;
    float m_fullCapacity;
    float m_soc;
    float m_temperature;
    uint16_t m_cycleCount;
    uint16_t m_swVersion;
    bool m_chargeMos;
    bool m_dischargeMos;

    bool m_got100;
    bool m_got101;
    bool m_got103;
};

#endif
