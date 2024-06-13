#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt64.h>
#include <std_srvs/Empty.h>

class JoyToggles
{
public:
  JoyToggles()
  {
    _private_nh = ros::NodeHandle("~");
    std::string joy_topic;
    _nh.param<std::string>("joy_topic", joy_topic, "joy");
    _joy_sub = _nh.subscribe(joy_topic, 1, &JoyToggles::joy_callback, this);
    _presses_pub = _nh.advertise<std_msgs::UInt64>("joy/toggles/presses", 1);
    _releases_pub = _nh.advertise<std_msgs::UInt64>("joy/toggles/releases", 1);
    _reset_service = _private_nh.advertiseService("reset", &JoyToggles::reset_callback, this);
  }

  ~JoyToggles()
  {
    _joy_sub.shutdown();
    _reset_service.shutdown();
    for (auto& pub_pair : _publishers)
    {
      pub_pair.first.shutdown();
      pub_pair.second.shutdown();
    }
    _presses_pub.shutdown();
    _releases_pub.shutdown();
  }

private:
  void joy_callback(const sensor_msgs::Joy::ConstPtr& joy_msg)
  {
    size_t buttons_num = joy_msg->buttons.size();
    if (buttons_num != _last_buttons.size())
      _reset_buttons_num(buttons_num);

    for (size_t i = 0; i < buttons_num; ++i)
    {
      int current = joy_msg->buttons[i];
      int last = _last_buttons[i];

      if (current > last)
      {
        _presses_counter[i].data += 1;
        _publishers[i].first.publish(_presses_counter[i]);
        auto msg = std_msgs::UInt64();
        msg.data = i;
        _presses_pub.publish(msg);
      }
      else if (current < last)
      {
        _releases_counter[i].data +=1 ;
        _publishers[i].second.publish(_releases_counter[i]);
        auto msg = std_msgs::UInt64();
        msg.data = i;
        _releases_pub.publish(msg);
      }
      _last_buttons[i] = current;
    }
  }

  void _reset_buttons_num(size_t new_size)
  {
    size_t current_size = _last_buttons.size();
    // If the current joystick number of buttons is greater than the last recorded,
    // add new publishers and init the buttons' states to 0.
    if (new_size > current_size)
    {
      _publishers.resize(new_size);
      _last_buttons.resize(new_size);
      _presses_counter.resize(new_size);
      _releases_counter.resize(new_size);
      for (size_t i = current_size; i < new_size; ++i)
      {
        std::string pressed_topic = "joy/toggles/" + std::to_string(i) + "/pressed";
        std::string released_topic = "joy/toggles/" + std::to_string(i) + "/released";
        _publishers[i].first = _nh.advertise<std_msgs::UInt64>(pressed_topic, 1, true);
        _publishers[i].first.publish(_presses_counter[i]);
        _publishers[i].second = _nh.advertise<std_msgs::UInt64>(released_topic, 1, true);
        _publishers[i].second.publish(_releases_counter[i]);
      }
      reset_counters();
    }

    // However, if the current joystick number of buttons is less than the last recorded,
    // release the spair publishers and re-init the buttons memory.
    else if (new_size < current_size)
    {
      _last_buttons.resize(new_size);
      _presses_counter.resize(new_size);
      _releases_counter.resize(new_size);
      for (size_t i = new_size; i < current_size; ++i)
      {
        _publishers[i].first.shutdown();
        _publishers[i].second.shutdown();
      }
      _publishers.resize(new_size);
      reset_counters();
    }
  }
    
  void reset_counters()
  {
    for(int i = 0; i < _last_buttons.size(); i++)
    {
      _last_buttons[i] = 0;
      _presses_counter[i].data = 0;
      _releases_counter[i].data = 0;
      _publishers[i].first.publish(_presses_counter[i]);
      _publishers[i].second.publish(_releases_counter[i]);
    }
  }

  bool reset_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    reset_counters();
    return true;
  }

  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  ros::Subscriber _joy_sub;
  ros::Publisher _presses_pub, _releases_pub;
  std::vector<std::pair<ros::Publisher, ros::Publisher>> _publishers;
  ros::ServiceServer _reset_service;

  std::vector<int> _last_buttons;
  std::vector<std_msgs::UInt64> _presses_counter, _releases_counter;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joy_toggle");
  JoyToggles joy_toggle;
  ros::spin();
  return 0;
}
