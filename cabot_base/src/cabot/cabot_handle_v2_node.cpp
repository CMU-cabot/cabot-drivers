/*******************************************************************************
 * Copyright (c) 2023  Miraikan and Carnegie Mellon University
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *******************************************************************************/

#include <memory>
#include <vector>
#include <map>
#include <string>

#include "cabot_handle_v2_node.hpp"

std::shared_ptr<CaBotHandleV2Node> node_;

CaBotHandleV2Node::CaBotHandleV2Node(const rclcpp::NodeOptions & options)
: rclcpp::Node("cabot_handle_v2_node", options), handle_(nullptr), button_keys_({}), event_pub_(
    nullptr) {
  event_pub_ = create_publisher<std_msgs::msg::String>("/cabot/event", 10);
  button_keys_ = declare_parameter("buttons", std::vector<std::string>{""});
  std::string button_keys_str = std::accumulate(
    button_keys_.begin(), button_keys_.end(), std::string(),
    [](const std::string & result, const std::string & key) {
      return result.empty() ? key : result + ", " + key;
    });
  callback = [this](const std::map<std::string, std::string>& msg){
    this->eventListener(msg);
  };
  //handle_ = std::make_shared<Handle>(shared_from_this(), callback, button_keys_);
  handle_ = std::make_shared<Handle>(std::dynamic_pointer_cast<CaBotHandleV2Node>(shared_from_this()), callback, button_keys_);

  RCLCPP_INFO(get_logger(), "buttons: %s", button_keys_str.c_str());
  bool no_vibration = declare_parameter("no_vibration", false);
  RCLCPP_INFO(get_logger(), "no_vibration = %s", no_vibration ? "true" : "false");
  if (!no_vibration) {
    notification_sub = create_subscription<std_msgs::msg::Int8>(
      "/cabot/notification", 10, [this](const std_msgs::msg::Int8::SharedPtr msg) {
        this->notificationCallback(msg);
      });
  }
  try {
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during spinning: %s", e.what());
    printStackTrace();
  }
}

void CaBotHandleV2Node::printStackTrace()
{
  void * array[10];
  size_t size;
  size = backtrace(array, 10);
  char ** symbols = backtrace_symbols(array, size);
  for (size_t i = 0; i < size; i++) {
    RCLCPP_ERROR(rclcpp::get_logger("cabot_handle_v2_node"), "StackTrace[%zu]: %s", i, symbols[i]);
  }
  free(symbols);
}

void CaBotHandleV2Node::notificationCallback(const std_msgs::msg::Int8::SharedPtr msg)
{
  if (msg) {
    std::string log_msg_ = "Received notification: " + std::to_string(msg->data);
    RCLCPP_INFO(this->get_logger(), log_msg_.c_str());
    node_->handle_->executeStimulus(msg->data);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Received nullptr message in notificationCallback");
  }
  rclcpp::Clock::SharedPtr clock = node_->get_clock();
  RCLCPP_INFO(
    this->get_logger(), "Node clock type (in notificationCallback): %d", clock->get_clock_type());
}

void CaBotHandleV2Node::eventListener(const std::map<std::string, std::string> & msg)
{
  std::shared_ptr<BaseEvent> event = nullptr;
  std::string msg_str;
  for (std::map<std::string, std::string>::const_iterator it = msg.begin(); it != msg.end(); ++it) {
    msg_str += "'" + it->first + "': " + it->second;
    if (std::next(it) != msg.end()) {
      msg_str += ", ";
    }
  }
  msg_str = "{" + msg_str + "}";
  RCLCPP_INFO(get_logger(), msg_str.c_str());
  if (msg_str.find("buttons") != std::string::npos) {
    int buttons = std::stoi(msg.at("buttons"));
    int count = std::stoi(msg.at("count"));
    event = std::make_shared<ClickEvent>(buttons, count);
  } else if (msg_str.find("button") != std::string::npos) {
    int button = std::stoi(msg.at("button"));
    bool up = (msg.at("up") == "True") ? true : false;
    bool hold = (msg.find("hold") != msg.end()) ? true : false;
    event = std::make_shared<ButtonEvent>(button, up, hold);
    std::shared_ptr<ButtonEvent> buttonEvent = std::dynamic_pointer_cast<ButtonEvent>(event);
    // button down confirmation
    if (buttonEvent && !buttonEvent->is_up()) {
      node_->handle_->executeStimulus(8);
    }
  } else if (msg_str.find("holddown") != std::string::npos) {
    int hold = std::stoi(msg.at("holddown"));
    event = std::make_shared<HoldDownEvent>(hold);
    std::shared_ptr<HoldDownEvent> holdDownEvent = std::dynamic_pointer_cast<HoldDownEvent>(event);
    // button hold down confirmation
    if (holdDownEvent) {
      node_->handle_->executeStimulus(9);
    }
  }
  if (event) {
    RCLCPP_INFO(get_logger(), event->toString().c_str());
    std_msgs::msg::String msg;
    msg.data = event->toString();
    event_pub_->publish(msg);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  node_ = std::make_shared<CaBotHandleV2Node>(rclcpp::NodeOptions());
  if (!node_) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to allocate memory for CaBotHandleV2Node .");
    return 1;
  }
  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(CaBotHandleV2Node)
