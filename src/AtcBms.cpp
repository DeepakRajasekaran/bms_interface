/*
 * Name: AtcBms.cpp
 * Author: Deepak Rajasekaran
 * Date: 10-Nov-2025
 * Version: 1.0
 * Description: Source file implementing ATC BMS CAN ROS1 node.
 */

#include "bms_interface/atc_bms_node.hpp"

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>

// ================= CAN IDs (from Python) =================
static constexpr uint32_t CELL_VOLTAGE_IDS[] = {
    0x0E640D09, 0x0E650D09, 0x0E660D09, 0x0E670D09,
    0x0E680D09, 0x0E690D09, 0x0E6A0D09
};

static constexpr uint32_t GENERAL_INFO_1 = 0x0A6D0D09;
static constexpr uint32_t GENERAL_INFO_2 = 0x0A6E0D09;
static constexpr uint32_t WAKEUP_CMD     = 0x0E64090D;
// =========================================================

AtcBmsNode::AtcBmsNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : can_socket_(-1),
      total_voltage_(0.0),
      current_(0.0),
      soc_(0.0),
      remaining_capacity_(0.0),
      full_capacity_(0.0),
      wakeup_interval_(0.25)
{
    pnh.param<std::string>("can_interface", can_iface_, "can0");
    pnh.param<std::string>("battery_topic", battery_topic_, "/bms/battery_state");
    pnh.param<double>("publish_rate", publish_rate_, 10.0);

    battery_pub_ =
        nh.advertise<sensor_msgs::BatteryState>(battery_topic_, 10);

    last_wakeup_ = ros::Time(0);

    if (!openCan())
    {
        ROS_FATAL("Failed to initialize CAN interface");
        ros::shutdown();
    }
}

AtcBmsNode::~AtcBmsNode()
{
    if (can_socket_ > 0)
        close(can_socket_);
}

// ================= CAN SETUP =================

bool AtcBmsNode::openCan()
{
    struct ifreq ifr{};
    struct sockaddr_can addr{};

    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
        ROS_ERROR("Failed to open CAN socket");
        return false;
    }

    std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ);
    ioctl(can_socket_, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("Failed to bind CAN socket");
        return false;
    }

    ROS_INFO("Connected to CAN interface: %s", can_iface_.c_str());
    return true;
}

// ================= MAIN LOOP =================

void AtcBmsNode::spin()
{
    ros::Rate rate(publish_rate_);

    while (ros::ok())
    {
        sendWakeup();
        readCanFrame();
        publishBattery();

        ros::spinOnce();
        rate.sleep();
    }
}

// ================= CAN HANDLING =================

void AtcBmsNode::sendWakeup()
{
    if ((ros::Time::now() - last_wakeup_).toSec() < wakeup_interval_)
        return;

    struct can_frame frame{};
    frame.can_id  = WAKEUP_CMD | CAN_EFF_FLAG;
    frame.can_dlc = 8;

    write(can_socket_, &frame, sizeof(frame));
    last_wakeup_ = ros::Time::now();
}

void AtcBmsNode::readCanFrame()
{
    struct can_frame frame{};
    int nbytes = read(can_socket_, &frame, sizeof(frame));
    if (nbytes > 0)
        parseFrame(frame);
}

void AtcBmsNode::parseFrame(const struct can_frame& frame)
{
    uint32_t id = frame.can_id & CAN_EFF_MASK;

    for (int i = 0; i < 7; i++)
    {
        if (id == CELL_VOLTAGE_IDS[i])
        {
            parseCellVoltage(i * 4, frame.data);
            return;
        }
    }

    if (id == GENERAL_INFO_1)
        parseGeneralInfo1(frame.data);
    else if (id == GENERAL_INFO_2)
        parseGeneralInfo2(frame.data);
}

// ================= PARSERS =================

void AtcBmsNode::parseCellVoltage(int start_index, const uint8_t* data)
{
    if (cell_voltages_.size() < start_index + 4)
        cell_voltages_.resize(start_index + 4, 0.0);

    for (int i = 0; i < 4; i++)
    {
        uint16_t raw =
            (data[i * 2] << 8) | data[i * 2 + 1];
        cell_voltages_[start_index + i] = raw / 1000.0;  // mV → V
    }
}

void AtcBmsNode::parseGeneralInfo1(const uint8_t* data)
{
    total_voltage_ =
        ((data[0] << 8) | data[1]) * 0.1;

    int16_t cur =
        (data[2] << 8) | data[3];
    current_ = cur * 0.1;

    remaining_capacity_ =
        ((data[4] << 8) | data[5]) * 0.1;

    full_capacity_ =
        ((data[6] << 8) | data[7]) * 0.1;
}

void AtcBmsNode::parseGeneralInfo2(const uint8_t* data)
{
    soc_ =
        ((data[0] << 8) | data[1]) * 0.1 / 100.0;
}

// ================= ROS PUBLISH =================

void AtcBmsNode::publishBattery()
{
    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.voltage      = total_voltage_;
    msg.current      = current_;
    msg.charge       = remaining_capacity_;
    msg.capacity     = full_capacity_;
    msg.percentage   = soc_;
    msg.cell_voltage = cell_voltages_;

    battery_pub_.publish(msg);
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