
#include <a05/a05.h>

namespace a05_driver
{
A05DriverNode::A05DriverNode()
{
  ros::NodeHandle private_node("~");
  private_node.param<string>("port", port_, "/dev/ttyUSB0");
  private_node.param<int>("baud", baud_, 9600);
  private_node.param<int>("rate", rate, 10);
  private_node.param<bool>("pub_float", pub_float_, true);
  private_node.param<bool>("pub_range", pub_range_, true);
  private_node.param<bool>("debug", debug_, false);
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 2000, 0, 2000, 0);
  try
  {
    this->serial_.open();
  }
  catch (serial::IOException& e)
  {
    ROS_ERROR_STREAM("Sonar Open Serial failed!");
  }
}

A05DriverNode::~A05DriverNode()
{
  this->serial_.close();
}

void A05DriverNode::pubTimerCallback(const ros::TimerEvent& event)
{
  std_msgs::Float32 sonar_data1, sonar_data2, sonar_data3, sonar_data4;
  sonar_data1.data = sonar_dist_01_;
  sonar_data2.data = sonar_dist_02_;
  sonar_data3.data = sonar_dist_03_;
  sonar_data4.data = sonar_dist_04_;

  sonar_01_.header.stamp = ros::Time::now();
  sonar_01_.header.frame_id = "sonar_01";
  sonar_01_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_01_.field_of_view = 0.1;
  sonar_01_.min_range = 0;
  sonar_01_.max_range = 4.5;
  sonar_01_.range = sonar_dist_01_;

  sonar_02_.header.stamp = ros::Time::now();
  sonar_02_.header.frame_id = "sonar_02";
  sonar_02_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_02_.field_of_view = 0.1;
  sonar_02_.min_range = 0;
  sonar_02_.max_range = 4.5;
  sonar_02_.range = sonar_dist_02_;

  sonar_03_.header.stamp = ros::Time::now();
  sonar_03_.header.frame_id = "sonar_03";
  sonar_03_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_03_.field_of_view = 0.1;
  sonar_03_.min_range = 0;
  sonar_03_.max_range = 4.5;
  sonar_03_.range = sonar_dist_03_;

  sonar_04_.header.stamp = ros::Time::now();
  sonar_04_.header.frame_id = "sonar_04";
  sonar_04_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_04_.field_of_view = 0.1;
  sonar_04_.min_range = 0;
  sonar_04_.max_range = 4.5;
  sonar_04_.range = sonar_dist_04_;

  if (pub_float_)
  {
    publisher_01.publish(sonar_data1);
    publisher_02.publish(sonar_data2);
    publisher_03.publish(sonar_data3);
    publisher_04.publish(sonar_data4);
  }
  if (pub_range_)
  {
    sonar_pub_01.publish(sonar_01_);
    sonar_pub_02.publish(sonar_02_);
    sonar_pub_03.publish(sonar_03_);
    sonar_pub_04.publish(sonar_04_);
  }
}

void A05DriverNode::readTimerCallback(const ros::TimerEvent& event)
{
  sonar_dist_01_ = max_sonar_dist_;
  sonar_dist_02_ = max_sonar_dist_;
  sonar_dist_03_ = max_sonar_dist_;
  sonar_dist_04_ = max_sonar_dist_;

  uint8_t payload[DL_DATA_LENGTH + 1];
  fetch_payload(payload);
  sonar_dist_01_ = payload[0] << 8 | payload[1] << 0;
  sonar_dist_01_ = sonar_dist_01_ / 1000;
  history_range_.pop_front();
  history_range_.push_back(sonar_dist_01_);
  sonar_dist_02_ = payload[2] << 8 | payload[3] << 0;
  sonar_dist_02_ = sonar_dist_02_ / 1000;
  history_range_.pop_front();
  history_range_.push_back(sonar_dist_02_);
  sonar_dist_03_ = payload[4] << 8 | payload[5] << 0;
  sonar_dist_03_ = sonar_dist_03_ / 1000;
  history_range_.pop_front();
  history_range_.push_back(sonar_dist_03_);
  sonar_dist_04_ = payload[6] << 8 | payload[7] << 0;
  sonar_dist_04_ = sonar_dist_04_ / 1000;
  history_range_.pop_front();
  history_range_.push_back(sonar_dist_04_);
}

void A05DriverNode::emergencyTimerCallback(const ros::TimerEvent& event)
{
  std_msgs::Bool emergency_range;
  auto min_it = std::min_element(history_range_.begin(), history_range_.end());

  if (debug_)
  {
    ROS_INFO("min_range is %f\n", *min_it);
  }
  if (*min_it <= 0.25)
  {
    if (debug_)
    {
      ROS_INFO("min_range is lower than %f\n", *min_it);
    }
    emergency_range.data = true;
  }
  else
    emergency_range.data = false;
  emergency_publisher.publish(emergency_range);
}

bool A05DriverNode::check_eq(uint8_t num)
{
  uint8_t buffer;
  this->serial_.read(&buffer, 1);
  if (buffer == num)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool A05DriverNode::check_sum(uint8_t* payload)
{
  uint8_t checksum = 0;
  for (int i = 0; i < DL_DATA_LENGTH; i++)
  {
    checksum += payload[i];
  }
  checksum += 0xff;
  return checksum == payload[DL_DATA_LENGTH];
}

void A05DriverNode::fetch_payload(uint8_t* payload)
{
  unsigned char state = 0;
  while (1)
  {
    switch (state)
    {
      case 0: {  // Header
        state = check_eq(DL_HEADER) ? 1 : 0;
        break;
      }
      case 1: {  // PAYLOAD
        size_t read_payload_size = this->serial_.read(payload, (int)DL_DATA_LENGTH + 1);
        state = read_payload_size == (DL_DATA_LENGTH + 1) ? 2 : 0;
        break;
      }
      case 2: {  // CHECKSUM
        state = check_sum(payload) ? 3 : 0;
        break;
      }
      case 3: {
        state = 0;
        return;
      }
      default: {
        state = 0;
        break;
      }
    }
  }
}

void A05DriverNode::run()
{
  if (pub_float_)
  {
    publisher_01 = nh_.advertise<std_msgs::Float32>("/sonar_01", 10);
    publisher_02 = nh_.advertise<std_msgs::Float32>("/sonar_02", 10);
    publisher_03 = nh_.advertise<std_msgs::Float32>("/sonar_03", 10);
    publisher_04 = nh_.advertise<std_msgs::Float32>("/sonar_04", 10);
  }
  if (pub_range_)
  {
    sonar_pub_01 = nh_.advertise<sensor_msgs::Range>("/sonar_range_01", 10);
    sonar_pub_02 = nh_.advertise<sensor_msgs::Range>("/sonar_range_02", 10);
    sonar_pub_03 = nh_.advertise<sensor_msgs::Range>("/sonar_range_03", 10);
    sonar_pub_04 = nh_.advertise<sensor_msgs::Range>("/sonar_range_04", 10);
  }
  emergency_publisher = nh_.advertise<std_msgs::Bool>("/close_range", 10);
  ros::Timer read_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &A05DriverNode::readTimerCallback, this);
  ros::Timer pub_timer_ = nh_.createTimer(ros::Duration(1.0 / 5), &A05DriverNode::pubTimerCallback, this);
  ros::Timer emergency_timer_ =
      nh_.createTimer(ros::Duration(1.0 / rate), &A05DriverNode::emergencyTimerCallback, this);
  ros::spin();
}

}  // namespace a05_driver

int main(int argc, char* argv[])
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "a05_node");
  std::deque<float> init_range(12, 0.25);
  std::queue<float> history_range_(init_range);
  a05_driver::A05DriverNode driver;
  driver.run();
  return 0;
}
