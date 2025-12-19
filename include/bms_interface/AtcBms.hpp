/*
 * Name: AtcBms.hpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.2
 * Description: Header for ATC BMS CAN ROS1 node with safe destructor.
 */

#pragma once

#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>

#include <linux/can.h>
#include <linux/can/raw.h>

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
    // ---------------- ROS ----------------
    ros::Publisher battery_pub_;
    ros::Subscriber charging_status_sub_;

    bool is_charging_ = false;

    // ---------------- CAN ----------------
    int can_socket_;
    std::string can_iface_;

    // ---------------- Params ----------------
    std::string battery_topic_;
    double publish_rate_;

    // ---------------- BMS Data ----------------
    std::vector<float> cell_voltages_;   // volts
    float total_voltage_;
    float current_;
    float soc_;                           // 0..1
    float remaining_capacity_;
    float full_capacity_;

    // ---------------- Wakeup ----------------
    ros::Time last_wakeup_;
    double wakeup_interval_;

    // ---------------- Methods ----------------
    bool openCan();
    void sendWakeup();
    void readCanFrame();
    void parseFrame(const struct can_frame& frame);
    void publishBattery();

    // ---------------- Parsers ----------------
    void parseCellVoltage(int start_index, const uint8_t* data);
    void parseGeneralInfo1(const uint8_t* data);
    void parseGeneralInfo2(const uint8_t* data);
    void chargeStatusCallback(const std_msgs::Bool::ConstPtr& msg);

};
