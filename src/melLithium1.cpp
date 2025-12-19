

// File: src/MelBms.cpp
/*
Name:      MelBms.cpp
Author:    Auto-generated from user-provided parser
Date:      2025-11-27
Version:   1.0
Description:
  Implementation of MelBms ROS1 node. Parses MEL CAN frames and publishes
  sensor_msgs::BatteryState. Thread-safe and guideline-compliant.
*/

#include "bms_interface/melLithium1.hpp"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

MelBms::MelBms(ros::NodeHandle& nh,
               const std::string& can_iface,
               const std::string& publish_topic)
    : m_nh(nh),
      m_batteryPub(),
      m_publishRate(1.0),
      m_canIface(can_iface),
      m_socketFd(-1),
      m_running(false),
      m_chargingCableConnected(false),
      m_chargingStatus(false),
      m_batteryLossStatus(false),
      m_batteryReady(false),
      m_dischargeContactorState(false),
      m_chargeContactorStatus(false),
      m_batteryTroubleFree(false),
      m_batterySeriousFailure(false),
      m_batteryNormalFailure(false),
      m_batteryAlarmFailure(false),
      m_soc(0.0f),
      m_current(0.0f),
      m_voltage(0.0f),
      m_temperature(0.0f)
{
    double rate = 1.0;
    m_nh.param<double>("publish_rate", rate, rate);
    m_publishRate = ros::Rate(rate);

    std::string topic = publish_topic;
    m_nh.param<std::string>("battery_topic", topic, topic);

    m_batteryPub = m_nh.advertise<sensor_msgs::BatteryState>(topic, 5);
}

MelBms::~MelBms()
{
    stop();
}

bool MelBms::start()
{
    if (!initCanSocket())
        return false;

    m_running = true;
    m_readerThread = std::thread(&MelBms::readLoop, this);
    m_publisherThread = std::thread(&MelBms::publishLoop, this);

    ROS_INFO("MelBms: started on %s", m_canIface.c_str());
    return true;
}

void MelBms::stop()
{
    m_running = false;

    if (m_readerThread.joinable())
        m_readerThread.join();
    if (m_publisherThread.joinable())
        m_publisherThread.join();

    if (m_socketFd >= 0) {
        ::close(m_socketFd);
        m_socketFd = -1;
    }

    ROS_INFO("MelBms: stopped");
}

bool MelBms::initCanSocket()
{
    m_socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socketFd < 0) {
        perror("MelBms: socket");
        return false;
    }

    struct ifreq ifr{};
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, m_canIface.c_str(), IFNAMSIZ - 1);

    if (ioctl(m_socketFd, SIOCGIFINDEX, &ifr) < 0) {
        perror("MelBms: SIOCGIFINDEX");
        ::close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    struct sockaddr_can addr{};
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_socketFd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("MelBms: bind");
        ::close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    return true;
}

void MelBms::readLoop()
{
    struct can_frame frame{};
    fd_set readfds;
    while (m_running && ros::ok()) {
        FD_ZERO(&readfds);
        FD_SET(m_socketFd, &readfds);
        struct timeval timeout{};
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int ret = select(m_socketFd + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(m_socketFd, &readfds)) {
            ssize_t nbytes = read(m_socketFd, &frame, sizeof(frame));
            if (nbytes > 0) {
                parseFrame(frame);
            }
        }
    }
}

void MelBms::parseFrame(const struct can_frame& frame)
{
    // get extended ID (handles both standard/extended safely)
    uint32_t id = frame.can_id & CAN_EFF_MASK;

    std::lock_guard<std::mutex> lock(m_mutex);

    if (id == 0x18FF28F4u) {
        uint8_t battery_status = frame.data[0];

        m_chargingCableConnected = (battery_status & 0x01) != 0;
        m_chargingStatus         = (battery_status & 0x02) != 0;
        m_batteryLossStatus      = (battery_status & 0x04) != 0;
        m_batteryReady           = (battery_status & 0x08) != 0;
        m_dischargeContactorState= (battery_status & 0x10) != 0;
        m_chargeContactorStatus  = (battery_status & 0x20) != 0;

        m_soc = static_cast<float>(frame.data[1]);

        int16_t rawCurrent = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
        m_current = (static_cast<float>(rawCurrent) - 5000.0f) * 0.1f;

        uint16_t rawVoltage = static_cast<uint16_t>((frame.data[5] << 8) | frame.data[4]);
        m_voltage = static_cast<float>(rawVoltage) * 0.1f;

        uint8_t battery_fault_class = frame.data[6];
        m_batteryTroubleFree = m_batterySeriousFailure = m_batteryNormalFailure = m_batteryAlarmFailure = false;
        switch (battery_fault_class) {
            case 0x00: m_batteryTroubleFree = true; break;
            case 0x01: m_batterySeriousFailure = true; break;
            case 0x02: m_batteryNormalFailure = true; break;
            case 0x03: m_batteryAlarmFailure = true; break;
            default: break;
        }
    }
    else if (id == 0x18B428F4u) {
        int sum = 0;
        int count = 0;
        for (int i = 0; i < 8; ++i) {
            float t = static_cast<float>(frame.data[i]) - 40.0f;
            if (t < 0.0f) t = 0.0f;
            sum += static_cast<int>(t);
            ++count;
        }
        if (count > 0)
            m_temperature = static_cast<float>(sum) / static_cast<float>(count);
    }
}

void MelBms::publishBatteryState()
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        msg.present = true;
        msg.voltage = m_voltage;
        msg.current = m_current;
        msg.charge = 0.0;
        msg.capacity = 0.0;
        msg.design_capacity = 0.0;
        msg.percentage = (m_soc >= 0.0f && m_soc <= 100.0f) ? (m_soc / 100.0f) : 0.0f;
        msg.temperature = m_temperature;
        msg.power_supply_status = m_chargingStatus ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                                                  : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

        msg.location = m_chargeContactorStatus ? "CHG_ON" : "CHG_OFF";
        msg.serial_number = m_dischargeContactorState ? "DSG_ON" : "DSG_OFF";
    }

    m_batteryPub.publish(msg);
}

void MelBms::publishLoop()
{
    while (m_running && ros::ok()) {
        publishBatteryState();
        m_publishRate.sleep();
    }
}

void MelBms::spin()
{
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    stop();
}

// main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "mel_bms_node");
    ros::NodeHandle nh("~");

    std::string iface = "can0";
    std::string topic = "/bms/battery_state";
    double publish_rate = 1.0;

    nh.param<std::string>("can_interface", iface, iface);
    nh.param<std::string>("battery_topic", topic, topic);
    nh.param<double>("publish_rate", publish_rate, publish_rate);

    MelBms node(nh, iface, topic);
    // override publish rate if provided
    // (constructor reads publish_rate param already; to change here we would modify the class or add setter)

    if (!node.start()) {
        ROS_ERROR("mel_bms_node: failed to start");
        return 1;
    }

    node.spin();
    return 0;
}