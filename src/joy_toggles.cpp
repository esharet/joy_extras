#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>

class JoyToggles
{
public:
  JoyToggles()
  {
    _joy_sub = _nh.subscribe("joy", 1, &JoyToggles::joy_callback, this);
    buttons_num_ = 0;
  }

private:
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    buttons_num_ = joy_msg->buttons.size();
    if (buttons_num_ != _last_buttons.size())
      _resize_publishers_vector(buttons_num_);

    for (size_t i = 0; i < buttons_num_; ++i)
    {
      int current = joy_msg->buttons[i];
      int last = _last_buttons[i];

      if (current > last) 
        _publishers[i].first.publish(_empty_msg);
      else if (current < last)
        _publishers[i].second.publish(_empty_msg);
      
      _last_buttons[i] = current;
    }
  }

  void _resize_publishers_vector(size_t new_size)
  {
    size_t current_size = _last_buttons.size();
    // If the current joystick number of buttons is greater than the last recorded,
    // add new publishers and init the buttons' states to 0.
    if (new_size > current_size)
    {
      _publishers.resize(new_size);
      _last_buttons.resize(new_size);
      for (size_t i = current_size; i < new_size; ++i)
      {
        std::string pressed_topic = "joy/toggles/" + std::to_string(i) + "/pressed";
        std::string released_topic = "joy/toggles/" + std::to_string(i) + "/released";
        _publishers[i].first = _nh.advertise<std_msgs::Empty>(pressed_topic, 1);
        _publishers[i].second = _nh.advertise<std_msgs::Empty>(released_topic, 1);
        _last_buttons[i] = 0;
      }
    }

    // However, if the current joystick number of buttons is less than the last recorded,
    // release the spair publishers and re-init the buttons memory.
    else if (new_size < current_size)
    {
      _last_buttons.resize(new_size);
      for (size_t i = new_size; i < current_size; ++i)
      {
        _publishers[i].first.shutdown();
        _publishers[i].second.shutdown();
      }
      _publishers.resize(new_size);
    }
  }

  ros::NodeHandle _nh;
  ros::Subscriber _joy_sub;
  std::vector<int> _last_buttons;
  std::vector<std::pair<ros::Publisher, ros::Publisher>> _publishers;
  size_t buttons_num_;
  std_msgs::Empty _empty_msg;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_toggle");
  JoyToggles joy_toggle;
  ros::spin();
  return 0;
}
