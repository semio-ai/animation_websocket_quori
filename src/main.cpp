#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING

#include <boost/bind.hpp>
#include <boost/bind/placeholders.hpp>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <iostream>
#include <memory>
#include <functional>

#include "json/json.h"

using namespace std;

typedef websocketpp::server<websocketpp::config::asio> server;

class animation_server
{
public:
  animation_server(boost::asio::io_service &service)
    : seq_(0)
  {
    s_.set_error_channels(websocketpp::log::elevel::all);
    s_.set_access_channels(websocketpp::log::alevel::none);
    s_.init_asio(&service);

    s_.set_message_handler(boost::bind(&animation_server::on_message, this, _1, _2));

    ros::NodeHandle nh;
    pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 5);
  }

  void run()
  {
    s_.listen(9004);
    s_.start_accept();
  }

private:
  void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg)
  {
    s_.send(hdl, msg->get_payload(), msg->get_opcode());

    const string payload = msg->get_payload();
    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(payload, root))
    {
      cerr << "Failed to parse JSON" << endl;
      return;
    }

    std::vector<std::string> joint_names;
    std::vector<double> joint_values;
    for(Json::ValueIterator it = root.begin(); it != root.end(); ++it) {
      const std::string &name = it.key().asString();
      joint_names.push_back(name);
      joint_values.push_back(it->asDouble());
    }

    const std::vector<double> zeros(joint_names.size(), 0);

    sensor_msgs::JointState jointState;
    jointState.header.seq = seq_++;
    jointState.header.stamp = ros::Time::now();
    jointState.name = joint_names;
    jointState.position = joint_values;
    jointState.velocity = zeros;
    jointState.effort = zeros;

    pub_.publish(jointState);
  }

  unsigned int seq_;
  server s_;
  ros::Publisher pub_;
};

boost::asio::io_service service;

void queue_spin_ros();

void spin_ros(const boost::system::error_code &)
{
  ros::spinOnce();
  queue_spin_ros();
}

void queue_spin_ros()
{
  boost::asio::deadline_timer t(service, boost::posix_time::milliseconds(10));
  t.async_wait(&spin_ros);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "animation_websocket_quori");

  animation_server server(service);
  server.run();

  queue_spin_ros();

  service.run();

  return 0;
}