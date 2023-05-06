#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <serial/serial.h>
#include <unistd.h>

#include <algorithm>
#include <vector>
#include <queue>

#define DL_HEADER                         0xff
#define DL_DATA_LENGTH                    8
#define DL_CHECKSUM_LENGTH                0x01

using namespace std;

class A05DriverNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit A05DriverNode(const rclcpp::NodeOptions & options);

  ~A05DriverNode();


private:
  // Set up ROS.
  string port_;
  int baud_, smooth_, rate;
  float sonar_dist_01_, sonar_dist_02_, sonar_dist_03_, sonar_dist_04_;
  sensor_msgs::msg::Range sonar_01_, sonar_02_, sonar_03_, sonar_04_;
  bool pub_float_, pub_range_, debug_; 

  serial::Serial serial_;
  float max_sonar_dist_;
  std::deque<float> history_range_ = std::deque<float> (12, 0.25);
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_01, publisher_02, publisher_03, publisher_04;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr sonar_pub_01, sonar_pub_02, sonar_pub_03, sonar_pub_04;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_publisher;
  rclcpp::TimerBase::SharedPtr read_timer_, pub_timer_, emergency_timer_;
  
  void readTimer();
  void pubTimer();
  void emergencyTimer();
  bool check_eq( uint8_t num);
  bool check_sum(uint8_t * payload);
  void fetch_payload( uint8_t* payload);
};

bool A05DriverNode::check_eq( uint8_t num)
{
  uint8_t buffer;
  this->serial_.read(&buffer, 1);
  if (buffer == num){
    return true;
  }else{
    return false;
  }
}

bool A05DriverNode::check_sum(uint8_t * payload)
{
  uint8_t checksum = 0;
  for (int i=0; i<DL_DATA_LENGTH; i++){
    checksum += payload[i];
  }
  checksum += 0xff;
  return checksum == payload[DL_DATA_LENGTH];
}

void A05DriverNode::fetch_payload( uint8_t* payload)
{
  unsigned char state = 0;
  while(1)
  {
    switch (state)
    {
      case 0:{ // Header
        state = check_eq(DL_HEADER) ? 1 : 0;  
        break;
      }case 1:{ // PAYLOAD
        size_t read_payload_size = this->serial_.read(payload, (int)DL_DATA_LENGTH + 1);
        state = read_payload_size == (DL_DATA_LENGTH + 1) ? 2 : 0;
        break;
      }case 2:{ // CHECKSUM
        state = check_sum(payload) ? 3 : 0;
        break;
      }case 3:{
        state = 0;
        return;
      }default:{
        state = 0;
        break;
      }
    }   
  }
}
