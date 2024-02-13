
#include "geometry_msgs/msg/twist.hpp"
#include "raspimouse_msgs/msg/switches.hpp"
#include "std_msgs/msg/int16.hpp"
#include "raspimouse_msgs/msg/leds.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"


template mros2::Publisher mros2::Node::create_publisher<std_msgs::msg::Int16>(std::string topic_name, int qos);
template void mros2::Publisher::publish(std_msgs::msg::Int16 &msg);

template mros2::Publisher mros2::Node::create_publisher<geometry_msgs::msg::Twist>(std::string topic_name, int qos);
template void mros2::Publisher::publish(geometry_msgs::msg::Twist &msg);

template mros2::Publisher mros2::Node::create_publisher<raspimouse_msgs::msg::Leds>(std::string topic_name, int qos);
template void mros2::Publisher::publish(raspimouse_msgs::msg::Leds &msg);



template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(raspimouse_msgs::msg::LightSensors*));
template void mros2::Subscriber::callback_handler<raspimouse_msgs::msg::LightSensors>(void *callee, const rtps::ReaderCacheChange &cacheChange);

template mros2::Subscriber mros2::Node::create_subscription(std::string topic_name, int qos, void (*fp)(raspimouse_msgs::msg::Switches*));
template void mros2::Subscriber::callback_handler<raspimouse_msgs::msg::Switches>(void *callee, const rtps::ReaderCacheChange &cacheChange);
