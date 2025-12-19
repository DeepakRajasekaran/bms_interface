// File: include/bms_interface/MelBms.hpp
/*
Name:      MelBms.hpp
Author:    Deepak Rajasekaran
Date:      2025-11-27
Version:   1.0
Description:
    Parses MEL BMS CAN frames (IDs 0x18FF28F4, 0x18B428F4)
  and publishes sensor_msgs::BatteryState.

*/

#ifndef BMS_INTERFACE_MELBMS_HPP
#define BMS_INTERFACE_MELBMS_HPP

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>

class MelBms
{
public:
    /**
     * @brief Construct a new MelBms node
     * @param nh ROS node handle (private recommended)
     * @param can_iface CAN interface name (default "can0")
     * @param publish_topic Topic to publish sensor_msgs::BatteryState
     */
    MelBms(ros::NodeHandle& nh,
           const std::string& can_iface = "can0",
           const std::string& publish_topic = "/bms/battery_state");

    /**
     * @brief Destroy the MelBms node (stops threads, closes socket)
     */
    ~MelBms();

    /**
     * @brief Initialize CAN and start threads
     * @return true on success
     */
    bool start();

    /**
     * @brief Stop threads and close socket
     */
    void stop();

    /**
     * @brief Main spin loop (keeps node alive)
     */
    void spin();

private:
    // non-copyable
    MelBms(const MelBms&) = delete;
    MelBms& operator=(const MelBms&) = delete;

    // CAN setup
    bool initCanSocket();

    // worker loops
    void readLoop();
    void publishLoop();

    // parser
    void parseFrame(const struct can_frame& frame);
    void publishBatteryState();

    // ROS
    ros::NodeHandle& m_nh;
    ros::Publisher m_batteryPub;
    ros::Rate m_publishRate;

    // CAN
    std::string m_canIface;
    int m_socketFd;

    // runtime
    std::atomic<bool> m_running;
    std::thread m_readerThread;
    std::thread m_publisherThread;

    // parsed state (protected by m_mutex)
    std::mutex m_mutex;

    bool m_chargingCableConnected;
    bool m_chargingStatus;
    bool m_batteryLossStatus;
    bool m_batteryReady;
    bool m_dischargeContactorState;
    bool m_chargeContactorStatus;

    bool m_batteryTroubleFree;
    bool m_batterySeriousFailure;
    bool m_batteryNormalFailure;
    bool m_batteryAlarmFailure;

    float m_soc;        // 0..100 (%)
    float m_current;    // A
    float m_voltage;    // V
    float m_temperature;// C (average)
};

#endif // BMS_INTERFACE_MELBMS_HPP
