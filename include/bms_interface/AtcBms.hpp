/*
 * Name: AtcBms.hpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 2.0
 * Description: ATC BMS CAN ROS1 node using
 *              dedicated CAN IO thread.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <thread>
#include <mutex>
#include <unordered_map>
#include <vector>
#include <string>
#include <cstdint>

class AtcBmsNode
{
public:
    AtcBmsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~AtcBmsNode();

    void spin();

private:
    // ================= ROS =================
    ros::Publisher  m_batteryPub;
    ros::Subscriber m_chargeStatusSub;

    // ================= CAN =================
    int         m_canSocket;
    std::string m_canInterface;

    std::thread m_canThread;
    bool        m_runCanThread;

    // Latest CAN frame per ID
    std::unordered_map<uint32_t, struct can_frame> m_canFrames;
    std::mutex m_canMutex;

    // ================= Params =================
    double m_publishRate;

    // ================= BMS Data =================
    std::vector<float> m_cellVoltages;
    float m_totalVoltage;
    float m_currentRaw;
    float m_soc;
    float m_remainingCapacity;
    float m_fullCapacity;

    bool  m_isCharging;

    // ================= Wakeup =================
    ros::Time m_lastWakeupTime;
    double    m_wakeupInterval;

    // ================= Methods =================
    bool openCan();
    void canThreadLoop();
    void sendWakeup();

    void parseStoredFrames();
    void publishBattery();

    // ================= Callbacks =================
    void chargeStatusCallback(const std_msgs::Bool::ConstPtr& msg);

    // ================= Parsers =================
    void parseCellVoltage(uint32_t canId, const uint8_t* data);
    void parseGeneralInfo1(const uint8_t* data);
    void parseGeneralInfo2(const uint8_t* data);
};
