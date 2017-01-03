/*******************************************************************************
 * Copyright (c) 2013 Frank Pagliughi <fpagliughi@mindspring.com>
   Edited by: 2016 Harshit Sureka <harshit.sureka@gmail.com>
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
#include "boost/thread.hpp"
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

std::string CLIENTID("mqttReceiver");

//Extend class mosquittopp from the /usr/include/mosquittopp.h file 
//This class provides all the basic functions for mqtt and 
//virtual callback functions that are implemented
class mqtt_bridge : public mosquittopp::mosquittopp
{
  private:
    //ros nodehandle to handle ROS Topics publishes and subscribes
    ros::NodeHandle nh_;

    //Message type for transporting images in ROS
    image_transport::ImageTransport it_;

    //The publisher for navdata on ROS topic
    ros::Publisher navdataPub_;

    //The publisher for images on ROS topic
    image_transport::Publisher imagePub_;

//		geometry_msgs::Publisher cmdVelPub_;


    //Variables to calculate the delay of a message. 
    
    //diff in time for navdata
    long long int diff_us_nav;
    
    //diff in time for video
    long long int diff_us_vid;
    
    //Count of navdata messages. Is reset to 0 after every 200 messages
    int navdataCount;
    
    //Count of video messages. Is reset to 0 after every 30 messages
    int videoCount;

    //The file streams for the delay output files for navdata and video
    std::fstream navdataFile;
    std::fstream videoFile;

    //Variables for handling the cmd_vel callback. Values between -1 and 1 
    //relating to left/right, front/back, up/down and turn movements.
    float old_left_right;
    float old_front_back;
    float old_up_down;
    float old_turn;

  public:
    //Variable to decide if delay files are to be written out.
    bool outputDelayFiles;

		//bool pingAcknowledgementReceived;
		

		bool firstPingAcknowledged;
		bool latestPingAcknowledged;
		bool timeToSendNextPing;

		uint32_t latestPingUsec;
		uint32_t latestPingSec;
		uint32_t latestPingID;
		boost::posix_time::ptime latestPosixTime;

    //The constructor
    mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh);

    //The Destructor
    ~mqtt_bridge();

    //Callback for when the mqtt client is connected
    void on_connect(int rc);

    //Callback for when the mqtt client receives a message on a subscribed topic
    void on_message(const struct mosquitto_message *message);

    //Callback for when the mqtt message succeeds in subscribing to a topic
    void on_subscribe(int mid, int qos_count, const int *granted_qos);

    //Custom Function: Initializes the delay files. Called only when outputDelayFiles = true.
    void setupDelayFiles();

    //Set the image and navdata publishers over ROS. Called in the constructor.
    void initPublishers();

    //Callback redirects here when a Navdata message is received over MQTT. This function packages the data received
    //over MQTT into a navMsg format for ROS. It then publishes that message out.
    void handleNavdata(const struct mosquitto_message *message);

    //Callback redirects here when a CompressedImage Message is received over MQTT. This function is under development.
    //void handleCompressedImage(const struct mosquitto_message *message);

    //Callback reidrects here when a uncompressedImage message is received over MQTT. The timestamp is extracted and then 
    //the file is packaged into an imageTransport message and sent as a ROS topic.
    void handleUncompressedImage(const struct mosquitto_message *message);

		void handleCmdVel(const struct mosquitto_message *message);

		void mqttPingFunction();

    //This is a callback for receiving a takeoff message on ROS. It is then sent over MQTT to be received by the sdk.
    //void takeOffMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a land message on ROS. It is then sent over MQTT to be received by the sdk.
    //void landMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a reset message on ROS. It is then sent over MQTT to be received by the sdk.
    //void resetMessageCallback(const std_msgs::Empty &msg);

    //This is a callback for receiving a cmd_vel message on ROS. It is then sent over MQTT to be received by the sdk.
    //void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
};


//Initialize the publishers that send the message over ROS topics. 
//This is called in the constructor.
void mqtt_bridge::initPublishers()
{
  //The publisher for ROS navdata on tum_ardrone/navdata
  navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("tum_ardrone/navdata", 200);

  //The publisher for ROS image messages on tum_ardrone/image
  imagePub_ = it_.advertise("tum_ardrone/image", 10); 
  
	//cmdVelPub_ = it_.advertise("tum_ardrone/cmd", 10); 
}


//The Constructor
//Intializes Variables, Intializes publishers, Connects the mqtt client.
mqtt_bridge::mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh) : 
  mosquittopp(id),
  nh_(nh), 
  it_(nh)
{
  int keepalive = 60;
  diff_us_nav = 0;
  diff_us_vid = 0;
  navdataCount = 0;
  videoCount = 0;
  outputDelayFiles = 0;

	firstPingAcknowledged = false;
	latestPingAcknowledged = false;

	latestPingUsec = 0;
	latestPingSec = 0;
	latestPingID = 0;
    
  old_left_right = -10.0;
  old_front_back = -10.0;
  old_up_down = -10.0;
  old_turn = -10.0;

  //initialize the navdata and img ros publishers
  initPublishers();

  //Connect this class instance to the mqtt host and port.
  connect(host, port, keepalive);
};

//Destructor
mqtt_bridge::~mqtt_bridge()
{
}

//Callback when the mqtt client successfully connects. rc = 0 means successful connection.
void mqtt_bridge::on_connect(int rc)
{
  printf("Connected with code %d.\n", rc);
}


void mqtt_bridge::handleCmdVel(const struct mosquitto_message *message)
{
	ROS_INFO("not handling CmdVel Msg right now\n");
}

//This is the function which handles the incoming images over MQTT.
//The images are prefixed by a timestamp which is extracted and then
//the image data is packed in a ROS Image_Transport message and published
//over the ROS Topic /ardrone/image_raw
void mqtt_bridge::handleUncompressedImage(const struct mosquitto_message *message)
{
  //Get the current time
  struct timeval tv;
  gettimeofday(&tv, NULL);

  //Get the time from the image message. This is stored as 2 uint32_ts. 1st is sec and 2nd is usec (microseconds)

  uint32_t org_sec; //the seconds in the message
  uint32_t org_usec; //the microseconds in the message

  //In the message that we received. We extract the first 4 and then the next 4 bytes into the time variables.
  //These calculations give us a warning because we are doing pointer arithmetic on void type pointer. But,
  //throughout systems sizeof(void) is returned as 1 and hence this should work well.
  //Quick solution is to cast as char* and then do arithmentic, but that seems not necessary
  memcpy(&org_sec, message->payload, 4);
  memcpy(&org_usec, message->payload + 4, 4);

  //If we are writing delays to the output file. Do So.
  if(outputDelayFiles)
  {
    double msgTime = (double)(org_sec) + (double)org_usec/1000000.0;
    double recvTime = (double)tv.tv_sec + (double)tv.tv_usec/1000000.0;
    videoFile << std::fixed << msgTime << ", ";
    videoFile << std::fixed << recvTime << ", ";
    videoFile << recvTime - msgTime << std::endl;
  }

  // Here we are calculating the average delay of the last 30 image messages received.
  uint32_t delaysec = (uint32_t)tv.tv_sec - org_sec;
  diff_us_vid = diff_us_vid + (delaysec)*1000000L + tv.tv_usec - org_usec;
  videoCount++;
  if(videoCount >= 30)
  {
    std::cout << "Average delay of last 30 video msgs: " << ((double)diff_us_vid/1000000L)/30.0 << " sec" << std::endl;
    videoCount = 0;
    diff_us_vid = 0;
  }
  /// 

  //Here we initialize the ROS Topic Image_Transport message that is to be sent out
  sensor_msgs::Image image_msg;
  image_msg.header.stamp = ros::Time::now();

  //We are hardcoding imageWidth and imageHeight values here since we aren't serializing the image data being sent from the sdk.
  uint32_t imageWidth = 640;
  image_msg.width = imageWidth;
  image_msg.height = 360;
  image_msg.encoding = "rgb8";
  image_msg.is_bigendian = false;
  image_msg.step = imageWidth * 3;

  //The imageData is contained after the 8th byte in the incoming message. First 8 bytes were time information.
  uint8_t* imgData = (uint8_t*)(message->payload+8);

  //The size of the image data is 8 less than the size of the incoming message. First 8 bytes were time information.
  unsigned long imgDataLen = message->payloadlen - 8;

  //Resize the data in the ros topic image transport message.
  image_msg.data.resize(imgDataLen);

  //If all is good, publish the data on the rostopic using the imagePub.
  if(imgData != NULL && imgDataLen != 0)
  {
    //copy image data from the incoming message to the data section of the outgoing message
    std::copy(imgData, imgData + imgDataLen, image_msg.data.begin());
    imagePub_.publish(image_msg);
  }
  else
  {
    std::cout << "Error in publishing image: Either imgData is NULL or the imgDataLen = 0" << std::endl;
  }
  return;
}
void mqtt_bridge::handleNavdata(const struct mosquitto_message *message)
{
  //Get the current time
  struct timeval tv;
  gettimeofday(&tv, NULL);

  //Initialize the binn object that we would use to extract the information from the incoming message which has been serialized
  //using this same binn library on the sending (SDK) side. 
  //The binn arranges data in a nice "key"->"value" format.
  binn* obj;
  obj = binn_open(message->payload);

  //Intialize the navdata message contained that would be sent over on the ROS topic.
  ardrone_autonomy::Navdata navMsg;
  navMsg.header.stamp = ros::Time::now();

  //Calculate the delay of the navdata message
  uint32_t org_sec = binn_object_uint32(obj, (char*)"time_sec");
  uint32_t org_usec = binn_object_uint32(obj, (char*)"time_usec");

  //If we are writing the delays to the delay file. Do So.
  if(outputDelayFiles)
  {
    double msgTime = (double)(org_sec) + (double)org_usec/1000000.0;
    double recvTime = (double)tv.tv_sec + (double)tv.tv_usec/1000000.0;
    navdataFile << std::fixed << msgTime << ", ";
    navdataFile << std::fixed << recvTime << ", ";
    navdataFile << recvTime - msgTime << std::endl;
  }

  //Here we calculate the average delay of the last 200 navdata messages received and print them out.
  uint32_t delaysec = (uint32_t)tv.tv_sec - org_sec;
  diff_us_nav = diff_us_nav + (delaysec)*1000000L + tv.tv_usec - org_usec;  
  navdataCount++;
  if(navdataCount >= 200)
  {
    std::cout << "Average delay of last 200 navdata msgs: " << ((double)diff_us_nav/1000000L)/200.0 << " sec" << std::endl;
    navdataCount = 0;
    diff_us_nav = 0;
  }
  ///////

  //Here we are extracting all the information from the incoming mqtt message. We are extracting the information
  //from the serialized binn structure. We are also performing operations that mirror the operations that 
  //are being perfromed by void ARDroneDriver::PublishNavdata(..) in the ardrone_autonomy ardrone_driver.cpp file in ROS

  navMsg.batteryPercent = binn_object_uint32(obj, (char*)"batteryPercent");
  navMsg.state = binn_object_uint32(obj, (char*)"state");

	navMsg.magX 			= binn_object_uint32(obj, (char*)"magX");
	navMsg.magY 			= binn_object_uint32(obj, (char*)"magY");
	navMsg.magZ 			= binn_object_uint32(obj, (char*)"magZ");
  navMsg.pressure 	= binn_object_uint32(obj, (char*)"pressure");
  navMsg.temp			 	= binn_object_uint32(obj, (char*)"temp");
	navMsg.wind_speed = binn_object_float(obj, (char*)"wind_speed");
	navMsg.wind_angle = binn_object_float(obj, (char*)"wind_angle");
	navMsg.wind_comp_angle = binn_object_float(obj, (char*)"wind_comp_angle");
  navMsg.rotX				= binn_object_float(obj, (char*)"rotX");
  navMsg.rotY 			= binn_object_float(obj, (char*)"rotY");
  navMsg.rotZ 			= binn_object_float(obj, (char*)"rotZ");
  navMsg.altd 			= binn_object_uint32(obj, (char*)"altd");
  navMsg.vx 				= binn_object_float(obj, (char*)"vx");
  navMsg.vy 				= binn_object_float(obj, (char*)"vy");
  navMsg.vz 				= binn_object_float(obj, (char*)"vz");
  navMsg.ax 				= binn_object_float(obj, (char*)"ax");
  navMsg.ay 				= binn_object_float(obj, (char*)"ay");
  navMsg.az 				= binn_object_float(obj, (char*)"az");
  navMsg.motor1 		= binn_object_uint32(obj, (char*)"motor1");
  navMsg.motor2 		= binn_object_uint32(obj, (char*)"motor2");
  navMsg.motor3 		= binn_object_uint32(obj, (char*)"motor3");
  navMsg.motor4 		= binn_object_uint32(obj, (char*)"motor4");
  navMsg.tm 				= binn_object_uint32(obj, (char*)"tm");

  //Send this message over on ROS Topic.
  navdataPub_.publish(navMsg);

  //Free the memory used by the binn pointer.
  binn_free(obj);

  return;
}

//When we receive a mqtt message, this callback is called. It just calls the responsible function
//depending on the topic of the mqtt message that was received.
void mqtt_bridge::on_message(const struct mosquitto_message *message)
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	uint32_t now_sec = (uint32_t)tv.tv_sec;
	uint32_t now_usec = (uint32_t)tv.tv_usec;
	if(!strcmp(message->topic, "/ardrone/navdata"))
	{
		handleNavdata(message);
	}
	else if(!strcmp(message->topic, "/ardrone/image"))
	{
		handleUncompressedImage(message);
	}
	else if(!strcmp(message->topic, "/mqtt/pings/response"))
	{
		if(!firstPingAcknowledged)
		{
			firstPingAcknowledged = true;
			latestPingAcknowledged = true;
		}

		boost::posix_time::ptime thisPosixTime	= boost::posix_time::microsec_clock::local_time();

		uint32_t thisMsgSec, thisMsgUsec, thisMsgID;
		memcpy(&thisMsgSec, message->payload, sizeof(uint32_t));
		memcpy(&thisMsgUsec, message->payload + sizeof(uint32_t), sizeof(uint32_t));
		memcpy(&thisMsgID, message->payload + 2*sizeof(uint32_t), sizeof(uint32_t));

		//std::cout << "Ack Received with ID: " << thisMsgID << " sec: " << thisMsgSec << " usec: " << thisMsgUsec << std::endl;
		boost::posix_time::time_duration diff = thisPosixTime - latestPosixTime;
		std::cout << "MQTT ping Delay is: " << diff.total_milliseconds() << " ms" << std::endl;
		//std::cout << "Msg was send with ID: " << latestPingID << " sec: " << latestPingSec << " usec: " << latestPingUsec << std::endl;
	

		//if(thisMsgID == latestPingID)
		//{
			latestPingAcknowledged = true;
		//  std::cout << "Delay is: " << ((now_sec - thisMsgSec)*1000000L + now_usec) - thisMsgUsec << " ms\n";
	//	}
	}
}

//Callback when the mosquitto library successfully subscribes to a topic
void mqtt_bridge::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  printf("Subscription succeeded.\n");
}

//The function that is called if we are outputting delays to a file. This function
//creates those files and writes the headers to them. The filenames contain
//the date and time of the run.
void mqtt_bridge::setupDelayFiles()
{
  struct tm *navdata_atm = NULL;
  struct timeval tv;
  char navdataFilename[100];
  char videoFilename[100];
  char timestring[100];
  gettimeofday(&tv,NULL);
  time_t temptime = (time_t)tv.tv_sec;
  navdata_atm = localtime(&temptime);
  strcpy(navdataFilename, "NavdataDelays");
  strcpy(videoFilename, "videoDelays");

  sprintf(timestring, "%04d%02d%02d_%02d%02d%02d.txt",
      navdata_atm->tm_year+1900, navdata_atm->tm_mon+1, navdata_atm->tm_mday,
      navdata_atm->tm_hour, navdata_atm->tm_min, navdata_atm->tm_sec);

  strcat(navdataFilename, timestring);
  strcat(videoFilename, timestring);

  std::cout << "Writing Navdata msg times and delays to " << navdataFilename << std::endl;
  navdataFile.open(navdataFilename, std::fstream::out | std::fstream::app);
  navdataFile << "MessageTime(s), ReceiveTime(s), Delay(s)" << std::endl;

  std::cout << "Writing video msg times and delays to " << videoFilename << std::endl;
  videoFile.open(videoFilename, std::fstream::out | std::fstream::app);
  videoFile << "MessageTime(s), ReceiveTime(s), Delay(s)" << std::endl;
}

void mqtt_bridge::mqttPingFunction()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);


	while(ros::ok())
	{
		latestPingID = latestPingID + 1;

		latestPingSec = (uint32_t)tv.tv_sec;
		latestPingUsec = (uint32_t)tv.tv_usec;
		latestPosixTime	= boost::posix_time::microsec_clock::local_time();
		uint8_t* bufToSend = (uint8_t*)malloc(3*sizeof(uint32_t));//
		memcpy(bufToSend, &latestPingSec, sizeof(uint32_t));
		memcpy(bufToSend + sizeof(uint32_t), &latestPingUsec, sizeof(uint32_t));
		memcpy(bufToSend + 2*sizeof(uint32_t), &latestPingID, sizeof(uint32_t));
		publish(NULL, "/mqtt/pings/request", 3*sizeof(uint32_t), bufToSend, 1);
		if(latestPingID > 10000)
		{
			latestPingID = 0;
		}
//		mqttBridge->timeToSendNextPing = false;

		sleep(1);
	}

//	if(mqttBridge->timeToSendNextPing) //first ping has been acknowledged and the last ping has also been acknowledged. Send next ping.
//	{
//		//Extract the seconds and microseconds from the current time
//	}
}

int main(int argc, char **argv)
{
  //Start with a new random client ID each time, so that prev messages aren't a hassle.
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  //Mandatory ROS INIT call for this file to be registered as a ROS NODE. 
  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle nodeHandle;

  //Initialize different variables that are to be read from the parameter file.
  std::string broker = "localhost";
  //std::string takeOffMsgTopic = "/ardrone/takeoff";
  //std::string landMsgTopic = "/ardrone/land";
  //std::string resetMsgTopic = "/ardrone/reset";
  //std::string cmdVelMsgTopic = "/cmd_vel";
  int brokerPort = 1883;

  //Read the variables from the parameter launch file. If the variable is not mentioned
  //in the parameter launch file, the defaults defined above are used. 
  //nodeHandle.getParam("/mqttReceiver/takeOffMsgTopic", takeOffMsgTopic);
  //nodeHandle.getParam("/mqttReceiver/landMsgTopic", landMsgTopic);
  //nodeHandle.getParam("/mqttReceiver/resetMsgTopic", resetMsgTopic);
  //nodeHandle.getParam("/mqttReceiver/cmdVelMsgTopic", cmdVelMsgTopic);
  nodeHandle.getParam("/mqttReceiver/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttReceiver/mqttBroker", broker);

  std::cout << "Connecting to " << broker << " at " << brokerPort << " port\n";
  
  //Initialize the mqttBridge class instance
  class mqtt_bridge *mqttBridge;

  mqttBridge->lib_init();

  mqttBridge = new mqtt_bridge(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  std::cout << "mqttBridge initialized..\n";


  //Here, we are subscribing to 4 ROS topics. These topics are published by tum_ardrone. Each topic is handled
  //by a separate callback which is the 3rd argument of the function calls below. On receipt of a message
  //the appropriate callback is called.
  //ros::Subscriber takeOffSub = nodeHandle.subscribe(takeOffMsgTopic, 1000, &mqtt_bridge::takeOffMessageCallback, mqttBridge);
  //ros::Subscriber landSub = nodeHandle.subscribe(landMsgTopic, 1000, &mqtt_bridge::landMessageCallback, mqttBridge);
  //ros::Subscriber resetSub = nodeHandle.subscribe(resetMsgTopic, 1000, &mqtt_bridge::resetMessageCallback, mqttBridge);
  //ros::Subscriber cmd_vel_sub = nodeHandle.subscribe(cmdVelMsgTopic, 1, &mqtt_bridge::CmdVelCallback, mqttBridge);
 
  /*****/
  //Get the variable from the parameter launch file whether or not to ouput delays to a file
  //If so, then setup the delay files and let the mqtt_bridge know that it needs to print values
  //to the files.
  bool delayFiles = false;
  nodeHandle.getParam("/mqttReceiver/outputDelayFiles", delayFiles);
  std::cout << "\nOutputDealyFiles set to " << delayFiles << std::endl;
  if(delayFiles)
  {
    //Setting this to true tells the mqttBridge class instance that whenever new navdata and image message is received, 
    //delay values need to be printed to those files.
    mqttBridge->outputDelayFiles = true;

    //This function sets up the output files and writes the headers to them.
    mqttBridge->setupDelayFiles();
  }
  /******/

  //Get the list of topics to subscribe to from the launch file
  std::vector<std::string> topicsList;
  ros::param::get("/mqttReceiver/topicsList",topicsList);

  //Iterate over the topics list and subscribe to each.
  //Each successful subscribe should print a "Subscribe Succeeded" message.
  for(int i =0 ; i < topicsList.size(); i++)
  {
    std::cout << "Subscribing to topic " << topicsList[i] << "\n"
      << "for client " << CLIENTID
      << " using QoS" << QOS << "\n\n";

    //Subscribe to each topic. On success the callback function on_subscribe(..) is called.
    mqttBridge->subscribe(NULL, topicsList[i].c_str());
  }
	mqttBridge->subscribe(NULL, "/ardrone/image");
	mqttBridge->subscribe(NULL, "/ardrone/cmd_vel");
	mqttBridge->subscribe(NULL, "/ardrone/navdata");
	mqttBridge->subscribe(NULL, "/mqtt/pings/response",1);

  int rc;

	boost::thread _mqttPingThread(&mqtt_bridge::mqttPingFunction, mqttBridge);
  //Now we have set everything up. We just need to loop around and act as the Bridge between ROS and MQTT.
  while(ros::ok()){

    //Pump all ROS callbacks. This function pushes all messages on the ROS subscribed topics and calls the appropriate callback
    //which were defined during the subscribe call.
    ros::spinOnce();

    //Pump all MQTT callbacks. This function pushes all messages on the MQTT Subscribed topics and calls the message_callback 
    //function defined for the mosquitto instance. The callback function handles different topics internally.
    rc = mqttBridge->loop();

    //If the first ping isnt acknowledged, keep pinging.
		


		if(rc){
			mqttBridge->reconnect();
    }
  }

	_mqttPingThread.join();

  ROS_INFO("Disconnecting MQTT....\n");

  //Cleanup the Connection
  mqttBridge->disconnect();

  //Cleanup the Mosquitto Library.
  mqttBridge->lib_cleanup();

  return 0;
}

