#include "attach_shelf/utils.hpp"
#include "rclcpp/logging.hpp"
#include <algorithm>
#include <chrono>
#include <map>
#include <sstream>

namespace utils {

size_t min_topics = 7;
size_t min_frame_count = 5;
int max_retries = 300;

std::string
pick_first_available_frame(const std::vector<std::string> &candidates,
                           const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
                           const rclcpp::Logger &logger) {
  std::vector<std::string> frames;

  // Retry loop: wait for TF tree to be populated
  for (int i = 0; i < max_retries; i++) {

    frames.clear();
    tf_buffer->_getFrameStrings(frames); // get the known TF frames

    RCLCPP_DEBUG(logger, "TF frame discovery attempt %d/%d: found %zu frames",
                 i + 1, max_retries, frames.size());

    if (frames.size() >= min_frame_count) {
      break;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  // Check if any candidate frame exists
  for (const auto &candidate : candidates) {
    RCLCPP_DEBUG(logger, "Checking candidate frame: %s", candidate.c_str());

    if (std::find(frames.begin(), frames.end(), candidate) != frames.end()) {
      RCLCPP_INFO(logger, "Selected TF frame: %s", candidate.c_str());
      return candidate;
    }
  }

  // Error message if no candidate found
  std::stringstream ss;
  ss << "None of the expected TF frames were found. Expected any of: ";
  for (auto &c : candidates)
    ss << c << " ";

  RCLCPP_ERROR(logger, "%s", ss.str().c_str());
  return "";
}

std::string
pick_first_available_topic(const std::vector<std::string> &candidates,
                           const rclcpp::Node *node,
                           const rclcpp::Logger &logger) {

  std::map<std::string, std::vector<std::string>> topics;

  // Re attempt to get topics names as its not stable
  for (int i = 0; i < max_retries; i++) {
    topics = node->get_topic_names_and_types();
    size_t count = topics.size();

    RCLCPP_DEBUG(node->get_logger(),
                 "Topic discovery attempt %d: found %zu topics", i + 1, count);

    if (count >= min_topics) {
      // Extract just the topic names
      std::vector<std::string> names;
      for (auto &pair : topics) {
        names.push_back(pair.first);
      }
      break;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(200));
  }

  for (const auto &candidate : candidates) {
    RCLCPP_DEBUG(logger, "topic ------- %s", candidate.c_str());
    if (topics.count(candidate)) {
      RCLCPP_INFO(logger, "Selected topic: %s", candidate.c_str());
      return candidate;
    }
  }

  std::stringstream ss;
  ss << "None of the expected topics found: ";
  for (auto &c : candidates)
    ss << c << " ";

  RCLCPP_ERROR(logger, "%s", ss.str().c_str());
  return "";
}

} // namespace utils
