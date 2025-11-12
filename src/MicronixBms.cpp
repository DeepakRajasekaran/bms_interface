/*
 * Name: MicronixBms.cpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.0
 * Description: Source file implementing Micronix BMS CAN ROS1 node.
 */

#include "bms_interface/MicronixBms.hpp"

MicronixBms::MicronixBms(ros::NodeHandle& nh)
    : m_loopRate(2),
      m_voltage(0), m_current(0), m_capacity(0),
      m_fullCapacity(0), m_soc(0), m_temperature(0),
      m_cycleCount(0), m_swVersion(0),
      m_chargeMos(false), m_dischargeMos(false),
      m_got100(false), m_got101(false), m_got103(false)
{
    m_batteryPub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);

    std::string iface = "can0";
    if (!initCan(iface))
    {
        ROS_ERROR("MicronixBms: Failed to initialize CAN interface %s", iface.c_str());
        ros::shutdown();
    }

    ROS_INFO("MicronixBms: Node initialized on %s", iface.c_str());
}

MicronixBms::~MicronixBms()
{
    try
    {
        if (m_canSocket >= 0)
        {
            close(m_canSocket);
            ROS_INFO("MicronixBms: CAN socket closed successfully.");
            m_canSocket = -1;
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("MicronixBms: Exception during destruction: %s", e.what());
    }
    catch (...)
    {
        ROS_ERROR("MicronixBms: Unknown error during destruction.");
    }
}


bool MicronixBms::initCan(const std::string& iface)
{
    m_canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_canSocket < 0)
    {
        perror("Socket");
        return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ);
    if (ioctl(m_canSocket, SIOCGIFINDEX, &ifr) < 0)
    {
        perror("SIOCGIFINDEX");
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(m_canSocket, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("Bind");
        return false;
    }

    return true;
}

bool MicronixBms::sendRequest(uint32_t id)
{
    struct can_frame frame {};
    frame.can_id = id;
    frame.can_dlc = 1;
    frame.data[0] = 0x5A;
    return (write(m_canSocket, &frame, sizeof(frame)) == sizeof(frame));
}

uint16_t MicronixBms::calculateCrc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

void MicronixBms::parse0x100(const uint8_t* d)
{
    m_voltage  = ((d[0] << 8) | d[1]) * 0.01;
    int16_t rawCurrent = (d[2] << 8) | d[3];
    m_current  = rawCurrent * 0.01;
    m_capacity = ((d[4] << 8) | d[5]) * 0.01;
    m_got100 = true;
}

void MicronixBms::parse0x101(const uint8_t* d)
{
    m_fullCapacity = ((d[0] << 8) | d[1]) * 0.01;
    m_cycleCount   = ((d[2] << 8) | d[3]);
    m_soc          = ((d[4] << 8) | d[5]) * 0.01;
    m_got101 = true;
}

void MicronixBms::parse0x103(const uint8_t* d)
{
    uint16_t mos = (d[0] << 8) | d[1];
    m_chargeMos = mos & 0x0001;
    m_dischargeMos = mos & 0x0002;
    m_swVersion = (d[4] << 8) | d[5];
    m_got103 = true;
}

void MicronixBms::readFrames()
{
    struct can_frame rx {};
    struct timeval timeout = {0, 100000};
    fd_set readSet;
    FD_ZERO(&readSet);
    FD_SET(m_canSocket, &readSet);
    select(m_canSocket + 1, &readSet, nullptr, nullptr, &timeout);

    while (FD_ISSET(m_canSocket, &readSet))
    {
        ssize_t nbytes = read(m_canSocket, &rx, sizeof(rx));
        if (nbytes < 0 || rx.can_dlc < 8)
            break;

        uint16_t rxCrc = (rx.data[6] << 8) | rx.data[7];
        uint16_t calcCrc = calculateCrc16(rx.data, 6);
        if (rxCrc != calcCrc)
            continue;

        if (rx.can_id == 0x100)
            parse0x100(rx.data);
        else if (rx.can_id == 0x101)
            parse0x101(rx.data);
        else if (rx.can_id == 0x103)
            parse0x103(rx.data);

        FD_ZERO(&readSet);
        FD_SET(m_canSocket, &readSet);
        select(m_canSocket + 1, &readSet, nullptr, nullptr, &timeout);
    }
}

void MicronixBms::publishBatteryMsg()
{
    if (!(m_got100 && m_got101))
        return;

    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.present = true;
    msg.voltage = m_voltage;
    msg.current = m_current;
    msg.charge = m_capacity;
    msg.capacity = m_fullCapacity;
    msg.percentage = m_soc;
    msg.temperature = m_temperature;
    msg.power_supply_status =
        (m_current >= 0) ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                         : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health =
        sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology =
        sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    msg.location = m_chargeMos ? "CHG_ON" : "CHG_OFF";
    msg.serial_number = m_dischargeMos ? "DSG_ON" : "DSG_OFF";

    m_batteryPub.publish(msg);
    m_got100 = m_got101 = m_got103 = false;
}

void MicronixBms::spin()
{
    while (ros::ok())
    {
        sendRequest(0x100);
        sendRequest(0x101);
        sendRequest(0x103);
        readFrames();
        publishBatteryMsg();
        ros::spinOnce();
        m_loopRate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micronix_bms_node");
    ros::NodeHandle nh;
    MicronixBms bmsNode(nh);
    bmsNode.spin();
    return 0;
}
