#include "mros2.h"
#include "mros2_log.h"

#include "cmsis_os.h"
#include "netif.h"
#include "netif_posix_add.h"

#include <algorithm>
#include <iostream>
#include <fstream>
#include <memory>
#include <chrono>
#include <functional>
#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "raspimouse_msgs/msg/leds.hpp"
#include "raspimouse_msgs/msg/light_sensors.hpp"
#include "raspimouse_msgs/msg/switches.hpp"

  using SensorsType = std::vector<int>;
  using namespace std::chrono_literals;
  enum SensorIndex
  {
    LEFT = 0,
    MID_LEFT,
    MID_RIGHT,
    RIGHT,
    SENSOR_NUM
  };
  mros2::Publisher pub_buzzer;
  mros2::Publisher pub_cmd_vel;
  mros2::Publisher pub_leds;

  mros2::Subscriber sub_lightsensors;
  mros2::Subscriber sub_switches;

  std_msgs::msg::Int16 buzzer_msg;
  geometry_msgs::msg::Twist cmd_vel_msg;
  raspimouse_msgs::msg::Leds leds_msg;

  static const int NUM_OF_SAMPLES = 4;

  raspimouse_msgs::msg::Switches switches;
  SensorsType present_sensor_values(SENSOR_NUM, 0);
  SensorsType sensor_line_values(SENSOR_NUM, 0);
  SensorsType sensor_field_values(SENSOR_NUM, 0);
  SensorsType line_thresholds(SENSOR_NUM, 0);
  SensorsType sampling_values(SENSOR_NUM, 0);
  std::vector<bool> line_is_detected_by_sensor(SENSOR_NUM, false);
  int sampling_count;
  bool line_values_are_sampled = false;
  bool field_values_are_sampled = false;
  bool can_publish_cmdvel = false;

bool sampling_is_done(){
    return line_values_are_sampled && field_values_are_sampled;
}

void set_line_thresholds() {
    if(!sampling_is_done()){
        return;
    }
    for (int i = 0; i < SENSOR_NUM; i++) {
        line_thresholds[i] = (sensor_line_values[i] + sensor_field_values[i]) / 2;
        printf("line_thresholds[%d] = %d\r\n", i, line_thresholds[i]);
    }

}

void update_line_detection() {
    for (int i = 0; i < SENSOR_NUM; i++) {
//        printf("present_sensor_values[%d] = %d\r\n", i, present_sensor_values[i]);
//        printf("present_senssor_values LEFT:%d MID_LEFT:%d MID_RIGHT:%d RIGHT:%d\r\n", present_sensor_values[LEFT], present_sensor_values[MID_LEFT], present_sensor_values[MID_RIGHT], present_sensor_values[RIGHT]);
        line_is_detected_by_sensor[i] = std::abs(present_sensor_values[i] > line_thresholds[i]);
    }

}

void beep_buzzer(int freq, int duration_ms){
    buzzer_msg.data = freq;
    printf("beep_buzzer %d %d\r\n", freq, duration_ms);
    pub_buzzer.publish(buzzer_msg);
    osDelay(duration_ms);
    buzzer_msg.data = 0;
    printf("beep_buzzer %d %d\r\n", buzzer_msg.data, duration_ms);
    pub_buzzer.publish(buzzer_msg);
}

void line_sampling() {
    beep_buzzer(1000, 500);
    sensor_line_values = present_sensor_values;
    printf("LEFT:%d MID_LEFT:%d MID_RIGHT:%d RIGHT:%d\r\n", sensor_line_values[LEFT], sensor_line_values[MID_LEFT], sensor_line_values[MID_RIGHT], sensor_line_values[RIGHT]);
    beep_buzzer(1000, 100);
    line_values_are_sampled = true;
    set_line_thresholds();
}

void field_sampling() {
    beep_buzzer(1000, 500);
    sensor_field_values = present_sensor_values;
    printf("LEFT:%d MID_LEFT:%d MID_RIGHT:%d RIGHT:%d\r\n", sensor_field_values[LEFT], sensor_field_values[MID_LEFT], sensor_field_values[MID_RIGHT], sensor_field_values[RIGHT]);
    beep_buzzer(1000, 100);
    field_values_are_sampled = true;
    set_line_thresholds();
}

void callback_light_sensors(raspimouse_msgs::msg::LightSensors *msg1){
    present_sensor_values[LEFT] = msg1->left;
    present_sensor_values[MID_LEFT] = msg1->forward_l;
    present_sensor_values[MID_RIGHT] = msg1->forward_r;
    present_sensor_values[RIGHT] = msg1->right;
    count_light_sensors = msg1->count;
    if (sampling_is_done()){
        update_line_detection();
    }
  tp = std::chrono::high_resolution_clock::now();
  tp_msec = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
  all_msec = tp_msec.count();
  std::ofstream outputfile("sub_output_mros2.txt", std::ios::app);
  if (!outputfile.is_open()) {
        std::cerr << "ファイルを開けませんでした。" << std::endl;
  }
  outputfile<<"ID:"<<count_light_sensors<<" "<<all_msec<<"\n";
  outputfile.close();
}

void callback_switches(raspimouse_msgs::msg::Switches *msg2){
    if(msg2->switch0){
        if(sampling_is_done() && !can_publish_cmdvel){
            //motor_on();
            beep_buzzer(1000, 500);
            can_publish_cmdvel = true;
        }else if (!can_publish_cmdvel){
            //motor_off();
            beep_buzzer(1000, 200);
            can_publish_cmdvel = false;
        }
    } else if (msg2->switch1){
        line_sampling();
        printf("line_sampling\r\n");
    } else if (msg2->switch2){
        field_sampling();
    }
    //printf("callback_switches %s %s %s\r\n", msg2->switch0 ? "true" : "false",msg2->switch1 ? "true" : "false", msg2->switch2 ? "true" : "false");
}

void publish_cmdvel_for_line_following(){
    geometry_msgs::msg::Twist msg;
    count_vel++;
    msg.count = count_vel;
    msg.linear.x = 0.0;
    msg.angular.z = 0.00;
    //printf("line_is_detected_by_sensor:%s %s %s %s\r\n", line_is_detected_by_sensor[0] ? "true" : "false", line_is_detected_by_sensor[1] ? "true" : "false", line_is_detected_by_sensor[2] ? "true" : "false", line_is_detected_by_sensor[3] ? "true" : "false");
    if(std::any_of(line_is_detected_by_sensor.begin(), line_is_detected_by_sensor.end(), [](bool v){return v;})){
        msg.linear.x = 0.08;
        msg.angular.z = 0.0;
        if(line_is_detected_by_sensor[LEFT]){
            msg.angular.z += 1.0;
        }
        if(line_is_detected_by_sensor[RIGHT]){
            msg.angular.z -= 1.0;
        }
        if(line_is_detected_by_sensor[MID_LEFT]){
            msg.angular.z += 0.5;
        }
        if(line_is_detected_by_sensor[MID_RIGHT]){
            msg.angular.z -= 0.5;
        }
    }
    printf("publish_cmdvel_for_line_following %f %f\r\n", msg.linear.x, msg.angular.z);
    tp = std::chrono::high_resolution_clock::now();
    tp_msec = std::chrono::duration_cast<std::chrono::microseconds>(tp.time_since_epoch());
    all_msec = tp_msec.count();
    std::ofstream outputfile("pub_output_mros2.txt", std::ios::app);
    if (!outputfile.is_open()) {
        std::cerr << "ファイルを開けませんでした。" << std::endl;
    }
    outputfile<<"ID:"<<count_vel<<" "<<all_msec<<"\n";
    outputfile.close();
    pub_cmd_vel.publish(msg);
    osDelay(10);
}

void indicate_line_detections() {
    raspimouse_msgs::msg::Leds msg;
    msg.led0 = line_is_detected_by_sensor[RIGHT];
    msg.led1 = line_is_detected_by_sensor[MID_RIGHT];
    msg.led2 = line_is_detected_by_sensor[MID_LEFT];
    msg.led3 = line_is_detected_by_sensor[LEFT];
    //printf("indicate_line_detections %s %s %s %s\r\n", msg.led0 ? "true" : "false",msg.led1 ? "true" : "false", msg.led2 ? "true" : "false", msg.led3 ? "true" : "false");
    pub_leds.publish(msg);
}

int main(int argc, char *argv[]){
    netif_posix_add(NETIF_IPADDR, NETIF_NETMASK);
    osKernelStart();
    mros2::init(0, NULL);
    MROS2_DEBUG("mROS 2 initialization is completed\r\n");
    mros2::Node node = mros2::Node::create_node("follower_on_mros2");

    pub_buzzer = node.create_publisher<std_msgs::msg::Int16>("buzzer", 10);
    pub_cmd_vel = node.create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    pub_leds = node.create_publisher<raspimouse_msgs::msg::Leds>("leds", 10);

    sub_lightsensors = node.create_subscription<raspimouse_msgs::msg::LightSensors>("light_sensors",10, callback_light_sensors);
    sub_switches = node.create_subscription<raspimouse_msgs::msg::Switches>("switches",10, callback_switches);

    count_vel = 0;
    count_light_sensors = 0;
    osDelay(100);

    MROS2_INFO("ready to pub/sub message\r\n");
//    buzzer_msg.data = 1000;
//    cmd_vel_msg.linear.x = 0.08;
//    cmd_vel_msg.angular.z = 0.8;
//    leds_msg.led0 = false;
//    leds_msg.led1 = false;
//    leds_msg.led2 = false;
//    leds_msg.led3 = false;
//    int count = 0;
    while(1){
        if (can_publish_cmdvel){
            publish_cmdvel_for_line_following();
        }
        indicate_line_detections();
    }
    mros2::spin();
    return 0;
}
