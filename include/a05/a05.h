#ifndef A05_H
#define A05_H

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

#include <serial/serial.h>
#include <tf/tf.h>
#include <unistd.h>

#include <string>
#include <algorithm>
#include <vector>
#include <queue>
#define DL_HEADER                         0xff
#define DL_DATA_LENGTH                    8
#define DL_CHECKSUM_LENGTH                0x01

using namespace std;

namespace a05_driver {
class A05DriverNode {
    public:
        A05DriverNode();
        ~A05DriverNode();
        void run(); 
        ros::Publisher publisher_01, publisher_02, publisher_03, publisher_04;
        ros::Publisher sonar_pub_01, sonar_pub_02, sonar_pub_03, sonar_pub_04;
        ros::Publisher emergency_publisher;

    private:
        sensor_msgs::Range sonar_01_, sonar_02_, sonar_03_, sonar_04_;
        bool pub_float_, pub_range_, debug_;

    private:
        ros::NodeHandle nh_;
        string port_;
        int baud_, rate;
        float sonar_dist_01_, sonar_dist_02_, sonar_dist_03_, sonar_dist_04_;
        serial::Serial serial_;
        float max_sonar_dist_;
        std::deque<float> history_range_ = std::deque<float> (12, 0.25);
        ros::NodeHandle nh;
        ros::Timer read_timer_;
        ros::Timer pub_timer_;
        ros::Timer emergency_timer_;
   
        void readTimerCallback(const ros::TimerEvent &);
        void pubTimerCallback(const ros::TimerEvent &);
        void emergencyTimerCallback(const ros::TimerEvent &);
        bool check_eq(uint8_t num);
        bool check_sum(uint8_t * payload);
        void fetch_payload(uint8_t* payload);
    };

}
#endif //A05_H
