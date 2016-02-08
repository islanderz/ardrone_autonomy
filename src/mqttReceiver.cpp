#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MQTTAsync.h"
#include <ardrone_autonomy/Navdata.h>
#include "zlib.h"
extern "C" {
#include "binn.h"
}


/****************************/

#define CLIENTID    "mqttReceiver"
#define QOS         1
#define TIMEOUT     10000L

/*******************************************************************************
 * Copyright (c) 2013 Frank Pagliughi <fpagliughi@mindspring.com>
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * and Eclipse Distribution License v1.0 which accompany this distribution. 
 *
 * The Eclipse Public License is available at 
 *    http://www.eclipse.org/legal/epl-v10.html
 * and the Eclipse Distribution License is available at 
 *   http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * Contributors:
 *    Frank Pagliughi - initial implementation and documentation
 *******************************************************************************/

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"
/////////////////////////////////////////////////////////////////////////////

class action_listener : public virtual mqtt::iaction_listener
{
	std::string name_;

	virtual void on_failure(const mqtt::itoken& tok) {
		std::cout << name_ << " failure";
		if (tok.get_message_id() != 0)
			std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
		std::cout << std::endl;
	}

	virtual void on_success(const mqtt::itoken& tok) {
		std::cout << name_ << " success";
		if (tok.get_message_id() != 0)
			std::cout << " (token: " << tok.get_message_id() << ")" << std::endl;
		if (!tok.get_topics().empty())
			std::cout << "\ttoken topic: '" << tok.get_topics()[0] << "', ..." << std::endl;
		std::cout << std::endl;
	}

public:
	action_listener(const std::string& name) : name_(name) {}
};

/////////////////////////////////////////////////////////////////////////////

class callback : public virtual mqtt::callback,
					public virtual mqtt::iaction_listener

{
	int nretry_;
	mqtt::async_client& cli_;
	action_listener& listener_;

  //ROS Topic Publisher
  ros::NodeHandle nh_;
  ros::Publisher navdataPub_;

	void reconnect() {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		mqtt::connect_options connOpts;
		connOpts.set_keep_alive_interval(20);
		connOpts.set_clean_session(true);

		try {
			cli_.connect(connOpts, nullptr, *this);
		}
		catch (const mqtt::exception& exc) {
			std::cerr << "Error: " << exc.what() << std::endl;
			exit(1);
		}
	}

	// Re-connection failure
	virtual void on_failure(const mqtt::itoken& tok) {
		std::cout << "Reconnection failed." << std::endl;
		if (++nretry_ > 5)
			exit(1);
		reconnect();
	}

	// Re-connection success
	virtual void on_success(const mqtt::itoken& tok) {
		std::cout << "Reconnection success" << std::endl;;
		cli_.subscribe("hello", QOS, nullptr, listener_);
	}

	virtual void connection_lost(const std::string& cause) {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;

		std::cout << "Reconnecting." << std::endl;
		nretry_ = 0;
		reconnect();
	}

	virtual void message_arrived(const std::string& topic, mqtt::message_ptr msg) {
		std::cout << "Message arrived on topic " << std::endl;
		std::cout << "\ttopic: '" << topic << "'" << std::endl;

    if(topic == "uas/ardrone1/navdata")
    {
      binn* obj;

      obj = binn_open((void*)(msg->get_payload()).data());

      uint32_t vbat_flying_percentage = binn_object_uint32(obj, (char*)"vbat_flying_percentage");
      uint32_t ctrl_state = binn_object_uint32(obj, (char*)"ctrl_state");
      int32_t pressure = binn_object_uint32(obj, (char*)"pressure");
      float theta = binn_object_float(obj, (char*)"theta");
      float phi = binn_object_float(obj, (char*)"phi");
      float psi = binn_object_float(obj, (char*)"psi");
      uint32_t altitude = binn_object_uint32(obj, (char*)"altitude");
      float vx = binn_object_float(obj, (char*)"vx");
      float vy = binn_object_float(obj, (char*)"vy");
      float vz = binn_object_float(obj, (char*)"vz");
      uint32_t motor1 = binn_object_uint32(obj, (char*)"motor1");
      uint32_t motor2 = binn_object_uint32(obj, (char*)"motor2");
      uint32_t motor3 = binn_object_uint32(obj, (char*)"motor3");
      uint32_t motor4 = binn_object_uint32(obj, (char*)"motor4");
      uint32_t tm = binn_object_uint32(obj, (char*)"tm");

      printf("Received navdata msg with angles: %f %f %f\n", theta, phi, psi);
      printf("Received navdata msg with velocities: %f %f %f\n", vx, vy, vz);
      printf("Received navdata msg with battery/ctrl_state/altd: %d/%d/%d\n", vbat_flying_percentage, ctrl_state, altitude);
      printf("\n");

      //NEED TO PUBLISH ON ROS TOPICS HERE

      //conversions from ardrone_driver.cpp
      ardrone_autonomy::Navdata navMsg;

      navMsg.header.stamp = ros::Time::now();
      navMsg.batteryPercent = vbat_flying_percentage;
      navMsg.state = ctrl_state >> 16;

      // positive means counterclockwise rotation around axis
      navMsg.rotX = phi / 1000.0;  // tilt left/right
      navMsg.rotY = theta / 1000.0;  // tilt forward/backward
      navMsg.rotZ = psi / 1000.0;  // orientation

      navMsg.altd = altitude;  // cm
      navMsg.vx = vx;  // mm/sec
      navMsg.vy = -vy;  // mm/sec
      navMsg.vz = -vz;  // mm/sec
      navMsg.motor1 = motor1;
      navMsg.motor2 = motor2;
      navMsg.motor3 = motor3;
      navMsg.motor4 = motor4;
      navMsg.tm = (tm & 0x001FFFFF) + (tm >> 21) * 1000000;

      navdataPub_.publish(navMsg);

      binn_free(obj);
      return;
    }
		//std::cout << "\t'" << msg->to_str() << "'\n" << std::endl;
	}

	virtual void delivery_complete(mqtt::idelivery_token_ptr token) {}

public:
	callback(mqtt::async_client& cli, action_listener& listener, ros::NodeHandle nh) 
				: cli_(cli), listener_(listener), nh_(nh) {}

  void setNavdataPublisher()
  {
    navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("ardrone/navdata", 200);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle nodeHandle;

  std::string broker = "tcp://unmand.io";
  std::string brokerPort = "1884";
  ros::param::get("/mqttReceiver/mqttBroker",broker);
  ros::param::get("/mqttReceiver/mqttBrokerPort",brokerPort);

  std::string address = broker + ":" + brokerPort;

  mqtt::async_client client(address.c_str(), CLIENTID);
  action_listener subListener("Subscription");

  callback cb(client, subListener,nodeHandle);
  client.set_callback(cb);

  //initialize the navdata publisher
  cb.setNavdataPublisher();
    

  mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  connOpts.set_clean_session(true);

  mqtt::itoken_ptr conntok = client.connect(connOpts);
  std::cout << "Waiting for the connection..." << std::flush;
  conntok->wait_for_completion();
  std::cout << "OK" << std::endl;

  std::vector<std::string> topicsList;
  ros::param::get("/mqttReceiver/topicsList",topicsList);
  for(int i =0 ; i < topicsList.size(); i++)
  {

    std::cout << "Subscribing to topic " << topicsList[i] << "\n"
      << "for client " << CLIENTID
      << " using QoS" << QOS << "\n\n"
      << "Press Q<Enter> to quit\n" << std::endl;

    client.subscribe(topicsList[i].c_str(), QOS, nullptr, subListener);
  }

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    loop_rate.sleep();
  }

  std::cout << "Disconnecting..." << std::flush;
  conntok = client.disconnect();
  conntok->wait_for_completion();
  std::cout << "Disconnect: OK" << std::endl;

  return 0;
  
  ROS_INFO("Exited Loop\n");
  ros::spin();

  return 0;
}
