/*******************************************************************************
 * Copyright (c) 2013 Frank Pagliughi <fpagliughi@mindspring.com>
   Edited by: Dec 2016 Harshit Sureka <harshit.sureka@gmail.com>
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

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <mosquittopp.h>
#include <image_transport/image_transport.h>
#include <sys/time.h>
#include <fstream>
#include <ardrone_autonomy/Navdata.h>
#include "zlib.h"
extern "C" {
#include "binn.h"
}
#define QOS         0
#define TIMEOUT     10000L
#define _EPS 1.0e-6
#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>

/////////////////////////////////////////////////////////////////////////////

std::string CLIENTID("mqttSender");

//Extend class mosquittopp from the /usr/include/mosquittopp.h file 
//This class provides all the basic functions for mqtt and 
//virtual callback functions that are implemented
class MQTTSender : public mosquittopp::mosquittopp
{
  private:
    //ros nodehandle to handle ROS Topics publishes and subscribes
    ros::NodeHandle nh_;

    //Message type for transporting images in ROS
    image_transport::ImageTransport it_;

  public:
    //The constructor
    MQTTSender(const char *id, const char *host, int port, ros::NodeHandle nh);

    //The Destructor
    ~MQTTSender();

    //Callback for when the mqtt client is connected
    void on_connect(int rc);

    //Callback for when the mqtt client receives a message on a subscribed topic
    void on_message(const struct mosquitto_message *message);

    //Callback for when the mqtt message succeeds in subscribing to a topic
    void on_subscribe(int mid, int qos_count, const int *granted_qos);

    //This is a callback for receiving a navdata msg over ROS. It is then sent over MQTT.
    void navdataMessageCallback(const ardrone_autonomy::Navdata &msg);

    //This is a callback for receiving an image msg over ROS. It is then sent over MQTT.
    void imageMessageCallback(const sensor_msgs::Image &msg);
    
    //This is a callback for receiving a takeoff message on ROS. It is then sent over MQTT to be received by the sdk.
    //void takeOffMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a land message on ROS. It is then sent over MQTT to be received by the sdk.
    //void landMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a reset message on ROS. It is then sent over MQTT to be received by the sdk.
    //void resetMessageCallback(const std_msgs::Empty &msg);

   // This is a callback for receiving a cmd_vel message on ROS. It is then sent over MQTT to be received by the sdk.
    void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
};

//The Constructor
//Intializes Variables, Intializes publishers, Connects the mqtt client.
MQTTSender::MQTTSender(const char *id, const char *host, int port, ros::NodeHandle nh) : 
  mosquittopp(id),
  nh_(nh), 
  it_(nh)
{
  int keepalive = 60;
  
	//Connect this class instance to the mqtt host and port.
  connect(host, port, keepalive);
};

//Destructor
MQTTSender::~MQTTSender()
{
}

//Callback when the mqtt client successfully connects. rc = 0 means successful connection.
void MQTTSender::on_connect(int rc)
{
  printf("Connected with code %d.\n", rc);
}


//When we receive a mqtt message, this callback is called. It just calls the responsible function
//depending on the topic of the mqtt message that was received.
void MQTTSender::on_message(const struct mosquitto_message *message)
{
//	std::cout << "Received an MQTT message on topic: " << message->topic << " of " << message->payloadlen << " bytes\n";
	if(!strcmp(message->topic,"ReceiverToSenderPing"))
	{
		publish(NULL, "SenderToReceiverPing", message->payloadlen, message->payload);
	}

}

//Callback when the mosquitto library successfully subscribes to a topic
void MQTTSender::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  printf("Subscription succeeded.\n");
}

void MQTTSender::navdataMessageCallback(const ardrone_autonomy::Navdata &msg)
{
	binn* obj;
	obj = binn_object();

	binn_object_set_uint32(obj, "batteryPercent", msg.batteryPercent);
	binn_object_set_uint32(obj, "state", msg.state);
	binn_object_set_int32(obj, "magX", msg.magX);
	binn_object_set_int32(obj, "magY", msg.magY);
	binn_object_set_int32(obj, "magZ", msg.magZ);
	binn_object_set_int32(obj, "pressure", msg.pressure);
	binn_object_set_int32(obj, "temp", msg.temp);
	binn_object_set_float(obj, "wind_speed", msg.wind_speed);
	binn_object_set_float(obj, "wind_angle", msg.wind_angle);
	binn_object_set_float(obj, "wind_comp_angle", msg.wind_comp_angle);

	binn_object_set_float(obj, "rotX", msg.rotX);
	binn_object_set_float(obj, "rotY", msg.rotY);
	binn_object_set_float(obj, "rotZ", msg.rotZ);
	binn_object_set_int32(obj, "altd", msg.altd);
	binn_object_set_float(obj, "vx", msg.vx);
	binn_object_set_float(obj, "vy", msg.vy);
	binn_object_set_float(obj, "vz", msg.vz);
	binn_object_set_float(obj, "ax", msg.ax);
	binn_object_set_float(obj, "ay", msg.ay);
	binn_object_set_float(obj, "az", msg.az);

	binn_object_set_uint32(obj, "motor1", msg.motor1);
	binn_object_set_uint32(obj, "motor2", msg.motor2);
	binn_object_set_uint32(obj, "motor3", msg.motor3);
	binn_object_set_uint32(obj, "motor4", msg.motor4);
	
	//sureka: ignoring tags_count, and other tag msgs.
	
	binn_object_set_uint32(obj, "tm", msg.tm);

	//Add the timestamp
	struct timeval tv;
	gettimeofday(&tv, NULL);
	binn_object_set_uint32(obj, "time_sec", (uint32_t)tv.tv_sec);
	binn_object_set_uint32(obj, "time_usec", (uint32_t)tv.tv_usec);

	//publish data from the binn using mqtt
	//ROS_INFO("I heard a navdata message of size %d over ROS. Sending out over MQTT.\n",binn_size(obj));
	publish(NULL, "/ardrone/navdata", binn_size(obj), (const uint8_t *)binn_ptr(obj));

	// release the buffer
	binn_free(obj);
}
void MQTTSender::imageMessageCallback(const sensor_msgs::Image &msg)
{
	unsigned long sendDataSize = 0;

	sendDataSize = msg.step * msg.height;

	//ROS_INFO("I heard a image message of size %d over ROS. Sending %d out over MQTT.\n",sendDataSize, sendDataSize + 8);
	//Get the current time
	struct timeval tv;
	gettimeofday(&tv, NULL);
	//Extract the seconds and microseconds from the current time
	uint32_t seconds = (uint32_t)tv.tv_sec;
	uint32_t useconds = (uint32_t)tv.tv_usec;

	//Create a new buffer that we would send. It contains the image data + 8 bytes (4*2 for each seconds and microseconds addition)
	uint8_t* bufWithTimestamp = (uint8_t*)malloc(sendDataSize + 8);

	//Copy the seconds field to the first 4 bytes
	memcpy(bufWithTimestamp, &seconds, 4);

	//Copy the microseconds field to the next 4 bytes
	memcpy(bufWithTimestamp + 4, &useconds, 4);

	//Copy the image data in rest of the buffer
//	memcpy(bufWithTimestamp + 8, msg.data.begin(), sendDataSize);
	std::copy(msg.data.begin(), msg.data.end(), bufWithTimestamp + 8);

	//Send the image buffer with Timestamp over MQTT.
	publish(NULL, "/ardrone/image", sendDataSize + 8, bufWithTimestamp);
}

//This function is called when a cmd_vel message is received over ROS topic. It publishes out the corresponding mqtt message.
//This message is usually sent by the tum_ardrone.
void MQTTSender::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
    binn* obj;
    obj = binn_object();

    binn_object_set_float(obj, "linearX", msg->linear.x);
    binn_object_set_float(obj, "linearY", msg->linear.y);
    binn_object_set_float(obj, "linearZ", msg->linear.z);
    binn_object_set_float(obj, "angularX", msg->angular.x);
    binn_object_set_float(obj, "angularY", msg->angular.y);
    binn_object_set_float(obj, "angularZ", msg->angular.z);

    //publish over the mqtt topic
    publish(NULL, "/ardrone/cmd_vel", binn_size(obj), (const uint8_t *)binn_ptr(obj));
    //ROS_INFO("I heard a cmd_vel Signal size %d. Sending it out over MQTT.\n", binn_size(obj));

    //free the binn object
    binn_free(obj);
}
int main(int argc, char **argv)
{
  //Start with a new random client ID each time, so that prev messages aren't a hassle.
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  //Mandatory ROS INIT call for this file to be registered as a ROS NODE. 
  ros::init(argc, argv, "mqttSender");
  ros::NodeHandle nodeHandle;

  //Initialize different variables that are to be read from the parameter file.
  std::string broker = "localhost";
  
  std::string cmdVelMsgTopic = "/cmd_vel";
	std::string imageMsgTopic = "/ardrone/image_raw";
	std::string navdataMsgTopic = "/ardrone/navdata";
  int brokerPort = 1883;

  //Read the variables from the parameter launch file. If the variable is not mentioned
  //in the parameter launch file, the defaults defined above are used. 
  nodeHandle.getParam("/mqttSender/cmdVelMsgTopic", cmdVelMsgTopic);
  nodeHandle.getParam("/mqttSender/imageMsgTopic", imageMsgTopic);
  nodeHandle.getParam("/mqttSender/navdataMsgTopic", navdataMsgTopic);
  nodeHandle.getParam("/mqttSender/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttSender/mqttBroker", broker);

  std::cout << "Connecting to " << broker << " at " << brokerPort << " port\n";
  
  //Initialize the mqttSender class instance
  class MQTTSender *mqttSender;

  mqttSender->lib_init();

  mqttSender = new MQTTSender(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  std::cout << "mqttSender initialized..\n";


  ros::Subscriber cmd_vel_sub = nodeHandle.subscribe(cmdVelMsgTopic, 1000, &MQTTSender::CmdVelCallback, mqttSender);
	ros::Subscriber image_sub = nodeHandle.subscribe(imageMsgTopic, 1000, &MQTTSender::imageMessageCallback, mqttSender);
  ros::Subscriber navdata_sub = nodeHandle.subscribe(navdataMsgTopic, 1000, &MQTTSender::navdataMessageCallback, mqttSender);
 
  /*****/
  //Get the variable from the parameter launch file whether or not to ouput delays to a file
  //If so, then setup the delay files and let the MQTTSender know that it needs to print values
  //to the files.

  //Get the list of topics to subscribe to from the launch file
  std::vector<std::string> topicsList;
  ros::param::get("/mqttSender/topicsList",topicsList);

  //Iterate over the topics list and subscribe to each.
  //Each successful subscribe should print a "Subscribe Succeeded" message.
  for(int i =0 ; i < topicsList.size(); i++)
  {
    std::cout << "Subscribing to topic " << topicsList[i] << "\n"
      << "for client " << CLIENTID
      << " using QoS" << QOS << "\n\n";

    //Subscribe to each topic. On success the callback function on_subscribe(..) is called.
    mqttSender->subscribe(NULL, topicsList[i].c_str());
  }
	mqttSender->subscribe(NULL, "ReceiverToSenderPing");

  int rc;

  //Now we have set everything up. We just need to loop around and act as the Bridge between ROS and MQTT.
  while(ros::ok()){

    //Pump all ROS callbacks. This function pushes all messages on the ROS subscribed topics and calls the appropriate callback
    //which were defined during the subscribe call.
    ros::spinOnce();

    //Pump all MQTT callbacks. This function pushes all messages on the MQTT Subscribed topics and calls the message_callback 
    //function defined for the mosquitto instance. The callback function handles different topics internally.
    rc = mqttSender->loop();

    //If the mqtt connection is giving any troubles. Try to reconnect.
    if(rc){
      mqttSender->reconnect();
    }
  }

  ROS_INFO("Disconnecting MQTT....\n");

  //Cleanup the Connection
  mqttSender->disconnect();

  //Cleanup the Mosquitto Library.
  mqttSender->lib_cleanup();

  return 0;
}

