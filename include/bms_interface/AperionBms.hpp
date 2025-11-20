/*
Name:      AperionBms.hpp
Author:    Deepak Rajasekaran
Date:      2025-11-18
Version:   1.0
Description: Header for AperionBms ROS1 node (APERION protocol parser).
             Follows ANSCER C++ Coding Guidelines (private members m_, comments, etc).
*/

#ifndef BMS_INTERFACE_APERIONBMS_HPP
#define BMS_INTERFACE_APERIONBMS_HPP

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>

#include <net/if.h>
#include <linux/can.h>

class AperionBms
{
public:
    /**
     * @brief Construct a new AperionBms node object
     * @param nh ROS node handle
     * @param iface CAN interface name (default "can0")
     */
    explicit AperionBms(ros::NodeHandle& nh, const std::string& iface = "can0");

    /**
     * @brief Destroy the AperionBms object and stops threads / closes socket
     */
    ~AperionBms();

    /**
     * @brief Main spin loop that handles sending queries, reading frames and publishing
     */
    void spin();

private:
    // Node / ROS
    ros::NodeHandle& m_nh;
    ros::Publisher m_batteryPub;
    ros::Rate m_loopRate;

    // CAN
    int m_socketFd;               // socket file descriptor
    std::string m_iface;          // CAN interface (e.g. "can0")

    // Parser state (protected by m_mutex when written/read from threads)
    std::mutex m_mutex;
    float m_voltage;
    float m_current;
    float m_percentage;
    float m_internalTemperature;
    float m_mcuTemperature;
    bool m_charge;
    bool m_discharge;

    // Threads / running flag
    std::thread m_readerThread;
    std::thread m_senderThread;
    std::atomic<bool> m_running;

    // Private methods
    /**
     * @brief Initialize and bind CAN socket to m_iface
     * @return true on success, false otherwise
     */
    bool initCan();

    /**
     * @brief Send a query (request) for a specific index
     * @param index index to query (0x00..0x14)
     */
    void sendQuery(uint8_t index);

    /**
     * @brief Thread function for sending periodic queries
     */
    void sendLoop();

    /**
     * @brief Thread function for reading frames from CAN socket
     */
    void readLoop();

    /**
     * @brief Parse a response frame according to APERION protocol
     * @param frame CAN frame read from socket
     */
    void parseResponse(const struct can_frame& frame);

    /**
     * @brief Publish current BatteryState (sensor_msgs::BatteryState)
     */
    void publishBatteryState();

    // Non-copyable
    AperionBms(const AperionBms&) = delete;
    AperionBms& operator=(const AperionBms&) = delete;
};

#endif // BMS_INTERFACE_APERIONBMS_HPP
