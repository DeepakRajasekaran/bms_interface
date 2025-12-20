/*
 * Name: AtcBms.cpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 2.0
 * Description: ATC BMS CAN ROS1 node with
 *              threaded CAN receive architecture.
 */

#include "bms_interface/AtcBms.hpp"

// ================= CAN IDs =================
static constexpr uint32_t CELL_VOLTAGE_IDS[] = {
    0x0E640D09, 0x0E650D09, 0x0E660D09, 0x0E670D09,
    0x0E680D09, 0x0E690D09, 0x0E6A0D09
};

static constexpr uint32_t GENERAL_INFO_1 = 0x0A6D0D09;
static constexpr uint32_t GENERAL_INFO_2 = 0x0A6E0D09;
static constexpr uint32_t WAKEUP_CMD     = 0x0E64090D;
// ===========================================

AtcBmsNode::AtcBmsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : m_canSocket(-1),
      m_runCanThread(true),
      m_totalVoltage(0.0f),
      m_currentRaw(0.0f),
      m_soc(0.0f),
      m_remainingCapacity(0.0f),
      m_fullCapacity(0.0f),
      m_isCharging(false),
      m_wakeupInterval(0.25)
{
    pnh.param<std::string>("can_interface", m_canInterface, "can0");
    pnh.param<double>("publish_rate", m_publishRate, 10.0);

    m_batteryPub =
        nh.advertise<sensor_msgs::BatteryState>("/bms/battery_state", 10);

    m_chargeStatusSub =
        nh.subscribe<std_msgs::Bool>(
            "charge_status", 10,
            &AtcBmsNode::chargeStatusCallback, this);

    if (!openCan())
    {
        ROS_FATAL("Failed to open CAN interface");
        ros::shutdown();
    }

    m_canThread = std::thread(&AtcBmsNode::canThreadLoop, this);
}

AtcBmsNode::~AtcBmsNode()
{
    m_runCanThread = false;
    if (m_canThread.joinable())
        m_canThread.join();

    if (m_canSocket >= 0)
        close(m_canSocket);
}

// ================= CAN SETUP =================

bool AtcBmsNode::openCan()
{
    struct ifreq ifr{};
    struct sockaddr_can addr{};

    m_canSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (m_canSocket < 0)
        return false;

    int flags = fcntl(m_canSocket, F_GETFL, 0);
    fcntl(m_canSocket, F_SETFL, flags | O_NONBLOCK);

    std::strncpy(ifr.ifr_name, m_canInterface.c_str(), IFNAMSIZ);
    ioctl(m_canSocket, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    return bind(m_canSocket,
                (struct sockaddr*)&addr,
                sizeof(addr)) == 0;
}

// ================= CAN THREAD =================

void AtcBmsNode::canThreadLoop()
{
    struct can_frame frame{};

    while (m_runCanThread)
    {
        sendWakeup();

        while (read(m_canSocket, &frame, sizeof(frame)) > 0)
        {
            std::lock_guard<std::mutex> lock(m_canMutex);
            m_canFrames[frame.can_id & CAN_EFF_MASK] = frame;
        }

        usleep(1000); // 1 ms
    }
}

void AtcBmsNode::sendWakeup()
{
    if ((ros::Time::now() - m_lastWakeupTime).toSec() < m_wakeupInterval)
        return;

    struct can_frame frame{};
    frame.can_id  = WAKEUP_CMD | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    write(m_canSocket, &frame, sizeof(frame));
    m_lastWakeupTime = ros::Time::now();
}

// ================= MAIN LOOP =================

void AtcBmsNode::spin()
{
    ros::Rate rate(m_publishRate);

    while (ros::ok())
    {
        parseStoredFrames();
        publishBattery();

        ros::spinOnce();
        rate.sleep();
    }
}

// ================= PARSING =================

void AtcBmsNode::parseStoredFrames()
{
    std::lock_guard<std::mutex> lock(m_canMutex);

    for (uint32_t id : CELL_VOLTAGE_IDS)
    {
        if (m_canFrames.count(id))
            parseCellVoltage(id, m_canFrames[id].data);
    }

    if (m_canFrames.count(GENERAL_INFO_1))
        parseGeneralInfo1(m_canFrames[GENERAL_INFO_1].data);

    if (m_canFrames.count(GENERAL_INFO_2))
        parseGeneralInfo2(m_canFrames[GENERAL_INFO_2].data);
}

void AtcBmsNode::parseCellVoltage(uint32_t canId, const uint8_t* data)
{
    int baseIndex = ((canId >> 16) & 0xFF) - 0x64;
    baseIndex *= 4;

    if (m_cellVoltages.size() < baseIndex + 4)
        m_cellVoltages.resize(baseIndex + 4, 0.0f);

    for (int i = 0; i < 4; i++)
    {
        uint16_t raw =
            (static_cast<uint16_t>(data[i * 2]) << 8) |
             static_cast<uint16_t>(data[i * 2 + 1]);

        m_cellVoltages[baseIndex + i] = raw / 1000.0f;
    }
}

void AtcBmsNode::parseGeneralInfo1(const uint8_t* data)
{
    m_totalVoltage =
        ((data[0] << 8) | data[1]) * 0.1f;

    int16_t currentInt =
        static_cast<int16_t>(
            (static_cast<uint16_t>(data[2]) << 8) |
             static_cast<uint16_t>(data[3])
        );

    m_currentRaw = currentInt * 0.1f;

    m_remainingCapacity =
        ((data[4] << 8) | data[5]) * 0.1f;

    m_fullCapacity =
        ((data[6] << 8) | data[7]) * 0.1f;
}

void AtcBmsNode::parseGeneralInfo2(const uint8_t* data)
{
    m_soc =
        ((data[0] << 8) | data[1]) * 0.1f / 100.0f;
}

// ================= CALLBACKS =================

void AtcBmsNode::chargeStatusCallback(
    const std_msgs::Bool::ConstPtr& msg)
{
    m_isCharging = msg->data;
}

// ================= PUBLISH =================

void AtcBmsNode::publishBattery()
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage      = m_totalVoltage;
    msg.current      = m_currentRaw;
    msg.charge       = m_remainingCapacity;
    msg.capacity     = m_fullCapacity;
    msg.percentage   = m_soc;
    msg.cell_voltage = m_cellVoltages;

    msg.power_supply_status =
        m_isCharging
            ? sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING
            : sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;

    m_batteryPub.publish(msg);
}

// ================= NODE ENTRY =================

int main(int argc, char** argv)
{
    ros::init(argc, argv, "atc_bms_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    AtcBmsNode node(nh, pnh);
    node.spin();

    return 0;
}
