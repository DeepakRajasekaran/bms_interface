/*
 * Name: AtcBms.cpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.0
 * Description: Source file implementing ATC BMS CAN ROS1 node.
 */

#include "bms_interface/AtcBms.hpp"

AtcBms::AtcBms(ros::NodeHandle& nh)
    : m_loopRate(2),
      m_voltage(0), m_current(0),
      m_capacity(0), m_fullCapacity(0),
      m_soc(0), m_temperature(0),
      m_cycleCount(0), m_gotInfo(false)
{
    m_batteryPub = nh.advertise<sensor_msgs::BatteryState>("battery_state", 10);

    std::string iface = "can0";
    if (!initCan(iface))
    {
        ROS_ERROR("AtcBms: Failed to initialize CAN interface %s", iface.c_str());
        ros::shutdown();
    }

    ROS_INFO("AtcBms: Node initialized on %s", iface.c_str());
}

AtcBms::~AtcBms()
{
    if (m_canSocket >= 0)
        close(m_canSocket);
}

bool AtcBms::initCan(const std::string& iface)
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

bool AtcBms::sendWakeup()
{
    struct can_frame frame {};
    frame.can_id = 0x0E64090D;  // Wakeup CMD (from ATC protocol)
    frame.can_dlc = 8;
    std::memset(frame.data, 0x00, 8);

    ssize_t nbytes = write(m_canSocket, &frame, sizeof(frame));
    return (nbytes == sizeof(frame));
}

void AtcBms::parseFrame(const struct can_frame& frame)
{
    // 0x0A6D0D09 → General Info 1 (Voltage, Current, Capacity)
    if (frame.can_id == 0x0A6D0D09 && frame.can_dlc >= 8)
    {
        uint16_t rawVolt = (frame.data[0] << 8) | frame.data[1];
        uint16_t rawCurr = (frame.data[2] << 8) | frame.data[3];
        uint16_t rawRemain = (frame.data[4] << 8) | frame.data[5];
        uint16_t rawFull = (frame.data[6] << 8) | frame.data[7];

        m_voltage = rawVolt * 0.1f;
        if (rawCurr > 32767)
            rawCurr -= 65536;
        m_current = rawCurr * 0.1f;
        m_capacity = rawRemain * 0.1f;
        m_fullCapacity = rawFull * 0.1f;
    }

    // 0x0A6E0D09 → General Info 2 (SOC, Cycles)
    else if (frame.can_id == 0x0A6E0D09 && frame.can_dlc >= 8)
    {
        uint16_t rawSoc = (frame.data[0] << 8) | frame.data[1];
        m_soc = rawSoc * 0.1f;
        m_cycleCount = (frame.data[4] << 8) | frame.data[5];
        m_gotInfo = true;
    }

    // 0x0A700D09 → General Info 4 (Temperature)
    else if (frame.can_id == 0x0A700D09 && frame.can_dlc >= 8)
    {
        int8_t temp = static_cast<int8_t>(frame.data[0]);
        m_temperature = static_cast<float>(temp);
    }
}

void AtcBms::readFrames()
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
        if (nbytes < 0)
            break;

        parseFrame(rx);

        FD_ZERO(&readSet);
        FD_SET(m_canSocket, &readSet);
        select(m_canSocket + 1, &readSet, nullptr, nullptr, &timeout);
    }
}

void AtcBms::publishBatteryMsg()
{
    if (!m_gotInfo)
        return;

    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.present = true;
    msg.voltage = m_voltage;
    msg.current = m_current;
    msg.charge = m_capacity;
    msg.capacity = m_fullCapacity;
    msg.percentage = m_soc / 100.0f;
    msg.temperature = m_temperature;

    msg.power_supply_status =
        (m_current >= 0) ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
                         : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    msg.power_supply_health =
        sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    msg.power_supply_technology =
        sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

    m_batteryPub.publish(msg);
    m_gotInfo = false;
}

void AtcBms::spin()
{
    while (ros::ok())
    {
        sendWakeup();
        readFrames();
        publishBatteryMsg();
        ros::spinOnce();
        m_loopRate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atc_bms_node");
    ros::NodeHandle nh;

    AtcBms bmsNode(nh);
    bmsNode.spin();

    return 0;
}
