#include <a05_driver_node.hpp>


A05DriverNode::A05DriverNode(const rclcpp::NodeOptions & node_options)
: Node("a05_driver", node_options), max_sonar_dist_(4.500)
{
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  // read parameter for serial
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 9600);
  rate = declare_parameter("rate", 10);
  pub_float_ = declare_parameter("pub_float", true);
  pub_range_ = declare_parameter("pub_range", true);
  // smooth参数，暂未使用
  smooth_ = declare_parameter("smooth", 4);
  // set parameter for serial
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 2000, 0, 2000, 0);
  this->serial_.open();
  std::deque<float> init_range(12, 0.25);
  std::queue<float> history_range_(init_range);

  // sonar data publisher
 if (pub_float_)
 {
    publisher_01 = create_publisher<std_msgs::msg::Float32>("~/sonar_dist_01_", 1);
    publisher_02 = create_publisher<std_msgs::msg::Float32>("~/sonar_dist_02_", 1);
    publisher_03 = create_publisher<std_msgs::msg::Float32>("~/sonar_dist_03_", 1);
    publisher_04 = create_publisher<std_msgs::msg::Float32>("~/sonar_dist_04_", 1);
 }
 
  // standard Ros Range data publisher
  if (pub_range_)
  {
    sonar_pub_01 = create_publisher<sensor_msgs::msg::Range>("~/sonar_01", 1);
    sonar_pub_02 = create_publisher<sensor_msgs::msg::Range>("~/sonar_02", 1);
    sonar_pub_03 = create_publisher<sensor_msgs::msg::Range>("~/sonar_03", 1);
    sonar_pub_04 = create_publisher<sensor_msgs::msg::Range>("~/sonar_04", 1);
  }
  
  //过小值0.25判断
  emergency_publisher = create_publisher<std_msgs::msg::Bool>("~/close_range", 1);

  read_timer_ = create_wall_timer(second_type(1.0 / rate), std::bind(&A05DriverNode::readTimer, this));

  pub_timer_ = create_wall_timer(second_type(1.0 / 5), std::bind(&A05DriverNode::pubTimer, this));

  emergency_timer_ = create_wall_timer(second_type(1.0 / rate), std::bind(&A05DriverNode::emergencyTimer, this));
}

A05DriverNode::~A05DriverNode() { this->serial_.close(); }

void A05DriverNode::pubTimer()
{
  std_msgs::msg::Float32 sonar_data1, sonar_data2, sonar_data3, sonar_data4;
  sonar_data1.data = sonar_dist_01_;
  sonar_data2.data = sonar_dist_02_;
  sonar_data3.data = sonar_dist_03_;
  sonar_data4.data = sonar_dist_04_;

  sonar_01_.header.stamp = rclcpp::Node::now();
  sonar_01_.header.frame_id = "sonar_01";
  sonar_01_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  sonar_01_.field_of_view = 0.1;
  sonar_01_.min_range = 0;
  sonar_01_.max_range = 4.5;
  sonar_01_.range = sonar_dist_01_;

  sonar_02_.header.stamp = rclcpp::Node::now();
  sonar_02_.header.frame_id = "sonar_02";
  sonar_02_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  sonar_02_.field_of_view = 0.1;
  sonar_02_.min_range = 0;
  sonar_02_.max_range = 4.5;
  sonar_02_.range = sonar_dist_02_;

  sonar_03_.header.stamp = rclcpp::Node::now();
  sonar_03_.header.frame_id = "sonar_03";
  sonar_03_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  sonar_03_.field_of_view = 0.1;
  sonar_03_.min_range = 0;
  sonar_03_.max_range = 4.5;
  sonar_03_.range = sonar_dist_03_;

  sonar_04_.header.stamp = rclcpp::Node::now();
  sonar_04_.header.frame_id = "sonar_04";
  sonar_04_.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  sonar_04_.field_of_view = 0.1;
  sonar_04_.min_range = 0;
  sonar_04_.max_range = 4.5;
  sonar_04_.range = sonar_dist_04_;

  if (pub_float_)
  {
    publisher_01->publish(sonar_data1);
    publisher_02->publish(sonar_data2);
    publisher_03->publish(sonar_data3);
    publisher_04->publish(sonar_data4);
  }
    if (pub_range_)
  {
    sonar_pub_01->publish(sonar_01_);
    sonar_pub_02->publish(sonar_02_);
    sonar_pub_03->publish(sonar_03_);
    sonar_pub_04->publish(sonar_04_);
  }
}

void A05DriverNode::readTimer()
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

void A05DriverNode::emergencyTimer()
{
  std_msgs::msg::Bool emergency_range;
  auto min_it = std::min_element(history_range_.begin(), history_range_.end());
  RCLCPP_INFO(this->get_logger(), "min_range is %f\n", *min_it);
  if (*min_it <= 0.25)
  {
    RCLCPP_INFO(this->get_logger(), "min_range is lower than %f\n", *min_it);
    emergency_range.data = true;
  }
  else emergency_range.data = false;
  emergency_publisher->publish(emergency_range);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(A05DriverNode)
