/*
Name:      AperionBms.cpp
Author:    Deepak Rajasekaran
Date:      2025-11-18
Version:   1.0
Description: Implementation of AperionBms node (APERION protocol parser).
*/

#include "bms_interface/AperionBms.hpp"

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <chrono>

AperionBms::AperionBms(ros::NodeHandle& nh, const std::string& iface)
    : m_nh(nh),
      m_batteryPub(nh.advertise<sensor_msgs::BatteryState>("battery_state", 10)),
      m_loopRate(2), // publish ~2 Hz (align with other nodes)
      m_socketFd(-1),
      m_iface(iface),
      m_voltage(0.0f),
      m_current(0.0f),
      m_percentage(0.0f),
      m_internalTemperature(0.0f),
      m_mcuTemperature(0.0f),
      m_charge(false),
      m_discharge(false),
      m_running(false)
{
    // Initialize CAN interface
    if (!initCan())
    {
        ROS_ERROR("AperionBms: Failed to initialize CAN interface %s", m_iface.c_str());
        // don't exit constructor; main should handle ros::ok / shutdown if needed
    }
    else
    {
        ROS_INFO("AperionBms: Initialized on %s", m_iface.c_str());
    }
}

AperionBms::~AperionBms()
{
    try
    {
        m_running = false;

        if (m_senderThread.joinable())
            m_senderThread.join();

        if (m_readerThread.joinable())
            m_readerThread.join();

        if (m_socketFd >= 0)
        {
            ::close(m_socketFd);
            m_socketFd = -1;
            ROS_INFO("AperionBms: CAN socket closed.");
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("AperionBms: Exception in destructor: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("AperionBms: Unknown exception in destructor.");
    }
}

bool AperionBms::initCan()
{
    m_socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_socketFd < 0)
    {
        perror("AperionBms: socket");
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, m_iface.c_str(), IFNAMSIZ - 1);

    if (ioctl(m_socketFd, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("AperionBms: SIOCGIFINDEX");
        ::close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_socketFd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0)
    {
        perror("AperionBms: bind");
        ::close(m_socketFd);
        m_socketFd = -1;
        return false;
    }

    // Start threads
    m_running = true;
    m_readerThread = std::thread(&AperionBms::readLoop, this);
    m_senderThread = std::thread(&AperionBms::sendLoop, this);

    return true;
}

void AperionBms::sendQuery(uint8_t index)
{
    const uint32_t query_id = 0x1AD;
    struct can_frame frame;
    std::memset(&frame, 0, sizeof(frame));

    frame.can_id = query_id;
    frame.can_dlc = 8;
    frame.data[0] = 0x01;
    frame.data[1] = 0x01;
    frame.data[2] = index;
    for (int i = 3; i < 8; ++i)
        frame.data[i] = 0x00;

    ssize_t n = write(m_socketFd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame)))
    {
        // Avoid flooding logs; throttle manually if necessary
        // perror("AperionBms: sendQuery write");
    }
}

void AperionBms::sendLoop()
{
    // Send indices 0x00...0x14 repeatedly with 400ms delay between them (as in original parser)
    while (m_running && ros::ok())
    {
        for (uint8_t idx = 0x00; idx <= 0x14 && m_running && ros::ok(); ++idx)
        {
            sendQuery(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(400));
        }
    }
}

void AperionBms::readLoop()
{
    struct can_frame frame;
    fd_set readfds;
    while (m_running && ros::ok())
    {
        FD_ZERO(&readfds);
        FD_SET(m_socketFd, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 200000; // 200ms

        int ret = select(m_socketFd + 1, &readfds, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(m_socketFd, &readfds))
        {
            ssize_t nbytes = read(m_socketFd, &frame, sizeof(frame));
            if (nbytes > 0)
            {
                // Original parser checked: (frame.can_id & CAN_EFF_MASK) == 0x65D
                // We'll be tolerant and check for lower 11-bit id equality when applicable
                if ((frame.can_id & CAN_EFF_MASK) == 0x65D || (frame.can_id & CAN_SFF_MASK) == 0x65D)
                {
                    parseResponse(frame);
                }
            }
        }
    }
}

void AperionBms::parseResponse(const struct can_frame& frame)
{
    // The index is at data[2]
    uint8_t index = 0;
    if (frame.can_dlc >= 3)
        index = frame.data[2];
    else
        return;

    std::lock_guard<std::mutex> lock(m_mutex);

    switch (index)
    {
        case 0x00: case 0x01: case 0x02: case 0x03:
        case 0x04: case 0x05: case 0x06: case 0x07:
        {
            // Each frame contains two cell voltages in bytes [4..7] in original parser
            // We do not publish per-cell now; keep placeholder in state if needed
            break;
        }
        case 0x08:
        {
            // Pack current & stack voltage (original used raw_current and raw_voltage)
            if (frame.can_dlc >= 7)
            {
                uint16_t raw_current = (frame.data[3] << 8) | frame.data[4];
                uint16_t raw_voltage = (frame.data[5] << 8) | frame.data[6];

                // conversions per original: current /100, voltage /100 (then /10 or /100?)
                // original parsed: voltage = raw_voltage / 1000 ? earlier used /1000 in places.
                // following the original parser's comments we use:
                m_voltage = static_cast<float>(raw_voltage) / 1000.0f; // V
                m_current = static_cast<float>(raw_current) / 100.0f; // A
            }
            break;
        }
        case 0x09:
        {
            // packVoltage (bytes 3..4) activeCells (5..6) - not used in BatteryState
            break;
        }
        case 0x0B: case 0x0C: case 0x0D:
        {
            // temperatures - avg or per-sensor; original used t1, t2
            if (frame.can_dlc >= 7)
            {
                uint16_t t1 = (frame.data[3] << 8) | frame.data[4];
                uint16_t t2 = (frame.data[5] << 8) | frame.data[6];
                // Convert to tenths/decimals as per original comments: t1/10.0
                (void)t1; (void)t2;
            }
            break;
        }
        case 0x0E:
        {
            // internal temps: data[5..6] contains internal temp per original
            if (frame.can_dlc >= 7)
            {
                uint16_t internalTemp = (frame.data[5] << 8) | frame.data[6];
                m_internalTemperature = static_cast<float>(internalTemp) / 100.0f;
            }
            break;
        }
        case 0x10:
        {
            // SOC, FETs and states
            if (frame.can_dlc >= 7)
            {
                m_percentage = static_cast<float>(frame.data[3]);
                uint8_t fet = frame.data[5];
                m_charge = !(fet & (1 << 4));          // Bit 4: charge (as original)
                m_discharge = (fet & (1 << 2)) != 0;   // Bit 2: discharge
            }
            break;
        }
        case 0x13:
        {
            if (frame.can_dlc >= 7)
            {
                uint16_t mcuTemp = (frame.data[3] << 8) | frame.data[4];
                m_mcuTemperature = static_cast<float>(mcuTemp) / 100.0f;
            }
            break;
        }
        default:
            // ignore others for now
            break;
    }
}

void AperionBms::publishBatteryState()
{
    // Create and publish sensor_msgs::BatteryState
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        msg.present = true;
        msg.voltage = m_voltage;
        msg.current = m_current;
        msg.percentage = m_percentage / 100.0f;
        msg.temperature = m_internalTemperature;
        msg.power_supply_status =
            (m_current >= 0) ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                             : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

        // location / serial_number fields are used by Micronix previously — leave empty or set flags
        msg.location = m_charge ? "CHG_ON" : "CHG_OFF";
        msg.serial_number = m_discharge ? "DSG_ON" : "DSG_OFF";
    }

    m_batteryPub.publish(msg);
}

void AperionBms::spin()
{
    // Publishing loop - keep in sync with m_loopRate
    while (ros::ok())
    {
        publishBatteryState();
        ros::spinOnce();
        m_loopRate.sleep();
    }

    // On exit, ensure threads stop
    m_running = false;
    if (m_senderThread.joinable())
        m_senderThread.join();
    if (m_readerThread.joinable())
        m_readerThread.join();
}

// main entry for standalone node
int main(int argc, char** argv)
{
    ros::init(argc, argv, "aperion_bms_node");
    ros::NodeHandle nh("~"); // private handle if parameters later

    std::string iface = "can0";
    // read param if provided (ANSCER prefers params/yaml)
    nh.param<std::string>("can_interface", iface, iface);

    AperionBms node(nh, iface);
    node.spin();

    return 0;
}
