#include <a05_driver_node.hpp>


A05DriverNode::A05DriverNode(const rclcpp::NodeOptions & node_options)
: Node("a05_driver", node_options), max_sonar_dist_(4.500)
{
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  // read parameter for serial
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 9600);
  rate = declare_parameter("rate", 10);
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
  // 01的距离
  publisher_01 = create_publisher<std_msgs::msg::Float32>("~/output/sonar_dist_01_", 1);
  // 02的距离
  publisher_02 = create_publisher<std_msgs::msg::Float32>("~/output/sonar_dist_02_", 1);
  // 03的距离
  publisher_03 = create_publisher<std_msgs::msg::Float32>("~/output/sonar_dist_03_", 1);
  // 04的距离
  publisher_04 = create_publisher<std_msgs::msg::Float32>("~/output/sonar_dist_04_", 1);
  //过小值0.25判断
  emergency_publisher = create_publisher<std_msgs::msg::Bool>("~/output/close_range", 1);

  read_timer_ = create_wall_timer(second_type(1.0 / rate), std::bind(&A05DriverNode::readTimer, this));

  pub_timer_ = create_wall_timer(second_type(1.0 / 5), std::bind(&A05DriverNode::pubTimer, this));

  emergency_timer_ = create_wall_timer(second_type(1.0 / rate), std::bind(&A05DriverNode::emergencyTimer, this));
}

A05DriverNode::~A05DriverNode() { this->serial_.close(); }

void A05DriverNode::pubTimer()
{
  std_msgs::msg::Float32 sonar_data1;
  sonar_data1.data = sonar_dist_01_;
  publisher_01->publish(sonar_data1);

  std_msgs::msg::Float32 sonar_data2;
  sonar_data2.data = sonar_dist_02_;
  publisher_02->publish(sonar_data2);

  std_msgs::msg::Float32 sonar_data3;
  sonar_data3.data = sonar_dist_03_;
  publisher_03->publish(sonar_data3);

  std_msgs::msg::Float32 sonar_data4;
  sonar_data4.data = sonar_dist_04_;
  publisher_04->publish(sonar_data4);
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
  RCLCPP_INFO(this->get_logger(), "min_range is %f\n",*min_it);
  if (*min_it <= 0.25)
  {
    RCLCPP_INFO(this->get_logger(), "min_range is lower than %f\n", history_range_.size() ,*min_it);
    emergency_range.data = true;
  }
  else emergency_range.data = false;
  emergency_publisher->publish(emergency_range);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(A05DriverNode)
