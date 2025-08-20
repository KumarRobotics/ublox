#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <ublox_gps/node.hpp>

// Declare the node globally to use in signal handler
std::shared_ptr<ublox_node::UbloxNode> node;

void signal_handler(int signal)
{
    RCLCPP_INFO(node->get_logger(), "SIGINT (%i) received, triggering shutdown.", signal);

    // Trigger the shutdown method of the UbloxNode
    auto current_state = node->get_current_state().id();
    if (rclcpp::ok() && current_state != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) 
    {
        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE); 
            current_state = node->get_current_state().id();
        }

        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
            node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
            current_state = node->get_current_state().id();
        }

        if (current_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            auto transitions = node->get_available_transitions();
            for (const rclcpp_lifecycle::Transition & t : transitions) {
                if (t.id() == lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN) {
                    node->trigger_transition(rclcpp_lifecycle::Transition(t.id()));  // shutdown
                    break;
                }
            }
        }
    }

    // Shutdown the ROS 2 node
    RCLCPP_INFO(node->get_logger(), "Shutting down UbloxNode...");
    rclcpp::shutdown();
}

int main(int argc, char** argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  // Create the UbloxNode instance
  node = std::make_shared<ublox_node::UbloxNode>(rclcpp::NodeOptions());
  
  // Register the signal handler for SIGINT
  std::signal(SIGINT, signal_handler);

  // Spin the node to process callbacks
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
	
	// Initialize UBlox 
	node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

	// Activate UBlox and start Watchdog
	node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  executor.spin();

  //rclcpp::shutdown();
  RCLCPP_INFO(node->get_logger(), "UbloxNode has been shut down gracefully.");

  return 0;
}
