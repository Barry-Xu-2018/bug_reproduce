#include "rclcpp/rclcpp.hpp"
#include <cstdio>

class TestParams : public rclcpp::Node {
	public:
		TestParams()
			: Node("test_params_node",
					rclcpp::NodeOptions()
					.allow_undeclared_parameters(false)
					.automatically_declare_parameters_from_overrides(true)) {
#if 0						
				rclcpp::Parameter declared_param;
				this->declare_parameter("declared_parameter", "Yes");
				this->get_parameter_or("declared_parameter", declared_param,
						rclcpp::Parameter("declared_parameter", "Yes"));
				RCLCPP_INFO(this->get_logger(), "declared_param: %s",
						declared_param.value_to_string().c_str());
#endif

				RCLCPP_INFO(
					this->get_logger(),
					"declared_param: %s",
					this->get_parameter("declared_parameter").get_parameter_value().get<std::string>().c_str());
				
				this->declare_parameter("declared_parameter", "Yes");
			}
};

// Code below is just to start the node
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TestParams>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
