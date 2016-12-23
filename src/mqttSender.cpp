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

    //The publisher for navdata on ROS topic
    //ros::Publisher navdataPub_;

    //The publisher for images on ROS topic
    //image_transport::Publisher imagePub_;


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

    //Custom Function: Initializes the delay files. Called only when outputDelayFiles = true.
    void setupDelayFiles();

		//void initPublishers();

    //Callback redirects here when a Navdata message is received over MQTT. This function packages the data received
    //over MQTT into a navMsg format for ROS. It then publishes that message out.
    void handleNavdata(const struct mosquitto_message *message);

    //Callback redirects here when a CompressedImage Message is received over MQTT. This function is under development.
    void handleCompressedImage(const struct mosquitto_message *message);

    //Callback reidrects here when a uncompressedImage message is received over MQTT. The timestamp is extracted and then 
    //the file is packaged into an imageTransport message and sent as a ROS topic.
    void handleUncompressedImage(const struct mosquitto_message *message);

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


//Initialize the publishers that send the message over ROS topics. 
//This is called in the constructor.
/*
void MQTTSender::initPublishers()
{
  //The publisher for ROS navdata on ardrone/navdata
  navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("ardrone/navdata", 200);

  //The publisher for ROS image messages on ardrone/image_raw
  imagePub_ = it_.advertise("ardrone/image_raw", 10); 
}
*/

//The Constructor
//Intializes Variables, Intializes publishers, Connects the mqtt client.
MQTTSender::MQTTSender(const char *id, const char *host, int port, ros::NodeHandle nh) : 
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
    
  old_left_right = -10.0;
  old_front_back = -10.0;
  old_up_down = -10.0;
  old_turn = -10.0;

  //initialize the navdata and img ros publishers
//  initPublishers();

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


//The callback when mqtt receives a compressed images. There are several
//errors in the uncompression, so this is temporarily being removed.
void MQTTSender::handleCompressedImage(const struct mosquitto_message *message)
{
  std::cout << "Compressed Images aren't fully handled yet....\n";
  
  //unsigned long imgDataLen;
  //uint8_t* imgData;
  //imgData = new uint8_t[20*message->payloadlen];
  //int ret_uncp = uncompress(imgData, &imgDataLen, (uint8_t*)message->payload, (unsigned long)message->payloadlen);
  //if (ret_uncp == Z_MEM_ERROR){printf("ERROR: uncompression memory error\n");return;}
  //else if (ret_uncp == Z_BUF_ERROR){printf("ERROR: uncompression buffer error\n");return;}
  //else if (ret_uncp == Z_DATA_ERROR){printf("ERROR: uncompression data error\n");return;}
  //else if (ret_uncp != Z_OK){printf("ERROR: uncompression error unknown\n");return;}

  //std::cout << "Compressed images are not yet handled properly" << std::endl;

  //std::cout << "Uncompressed from " << message->payloadlen << " to " << imgDataLen << " bytes" << std::endl;

  return;
}

//This is the function which handles the incoming images over MQTT.
//The images are prefixed by a timestamp which is extracted and then
//the image data is packed in a ROS Image_Transport message and published
//over the ROS Topic /ardrone/image_raw
void MQTTSender::handleUncompressedImage(const struct mosquitto_message *message)
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
    //magePub_.publish(image_msg);
  }
  else
  {
    std::cout << "Error in publishing image: Either imgData is NULL or the imgDataLen = 0" << std::endl;
  }
  return;
}
void MQTTSender::handleNavdata(const struct mosquitto_message *message)
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

  navMsg.batteryPercent = binn_object_uint32(obj, (char*)"vbat_flying_percentage");
  uint32_t ctrl_state = binn_object_uint32(obj, (char*)"ctrl_state");
  navMsg.state = ctrl_state >> 16;

  //it seems like there are some unit conversions and nomenclature adaptations that need to be done
  //on the navdata for it to be acknowledged by the tum_ardrone.

  navMsg.pressure = binn_object_uint32(obj, (char*)"pressure");
  navMsg.rotX = (binn_object_float(obj, (char*)"phi")) / 1000.0;
  navMsg.rotY = -1*(binn_object_float(obj, (char*)"theta")) / 1000.0;
  navMsg.rotZ = -1*(binn_object_float(obj, (char*)"psi")) / 1000.0;
  navMsg.altd = binn_object_uint32(obj, (char*)"altitude");
  navMsg.vx = binn_object_float(obj, (char*)"vx");
  navMsg.vy = -1*binn_object_float(obj, (char*)"vy");
  navMsg.vz = -1*binn_object_float(obj, (char*)"vz");
  navMsg.ax = (binn_object_float(obj, (char*)"ax")) / 1000.0;
  navMsg.ay = -1*(binn_object_float(obj, (char*)"ay")) / 1000.0;
  navMsg.az = -1*(binn_object_float(obj, (char*)"az")) / 1000.0;
  navMsg.motor1 = binn_object_uint32(obj, (char*)"motor1");
  navMsg.motor2 = binn_object_uint32(obj, (char*)"motor2");
  navMsg.motor3 = binn_object_uint32(obj, (char*)"motor3");
  navMsg.motor4 = binn_object_uint32(obj, (char*)"motor4");
  uint32_t tm = binn_object_uint32(obj, (char*)"tm");
  navMsg.tm = (tm & 0x001FFFFF) + (tm >> 21) * 1000000;

  //Send this message over on ROS Topic.
  //navdataPub_.publish(navMsg);

  //Free the memory used by the binn pointer.
  binn_free(obj);

  return;
}

//When we receive a mqtt message, this callback is called. It just calls the responsible function
//depending on the topic of the mqtt message that was received.
void MQTTSender::on_message(const struct mosquitto_message *message)
{
  if(!strcmp(message->topic, "uas/uav1/navdata"))
  {
    handleNavdata(message);
  }
  else if(!strcmp(message->topic, "uas/uav1/uncompressedImageStream"))
  {
    handleUncompressedImage(message);
  }
  else if(!strcmp(message->topic, "uas/uav1/compressedImageStream"))
  {
    handleCompressedImage(message);
  }
}

//Callback when the mosquitto library successfully subscribes to a topic
void MQTTSender::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  printf("Subscription succeeded.\n");
}

//The function that is called if we are outputting delays to a file. This function
//creates those files and writes the headers to them. The filenames contain
//the date and time of the run.
void MQTTSender::setupDelayFiles()
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

//This function is called when a takeoff message is received over ROS topic. It publishes out the corresponding mqtt message.
//void MQTTSender::takeOffMessageCallback(const std_msgs::Empty &msg)
//{
//  ROS_INFO("I heard a takeoff Signal. Sending it out over MQTT.\n");
// std::string takeOff = "takeoff1";
//  publish(NULL, "/ardrone/takeoff",  takeOff.length() , (const uint8_t *)takeOff.data());
//}

    
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
//	int ret = mosquitto_publish (navmosq, NULL, "uas/uav1/navdata", binn_size(obj), binn_ptr(obj), 0, false);
	ROS_INFO("I heard a navdata message of size %d over ROS. Sending out over MQTT.\n",binn_size(obj));
	publish(NULL, "/ardrone/navdata", binn_size(obj), (const uint8_t *)binn_ptr(obj));
//	if (ret)
//	{
//		fprintf (stderr, "NavClient: Can't publish to Mosquitto server\n");
//	}

	// release the buffer
	binn_free(obj);
}
void MQTTSender::imageMessageCallback(const sensor_msgs::Image &msg)
{
	unsigned long sendDataSize = 0;

	sendDataSize = msg.step * msg.height;

	ROS_INFO("I heard a image message of size %d over ROS. Sending %d out over MQTT.\n",sendDataSize, sendDataSize + 8);
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
//	int ret = mosquitto_publish (vidmosq, NULL, "uas/uav1/uncompressedImageStream", sendDataSize + 8, bufWithTimestamp, 0, false);
//	if (ret)
//	{
//		fprintf (stderr, "VidClient: Can't publish to Mosquitto server\n");
//	}


}

//This function is called when a land message is received over ROS topic. It publishes out the corresponding mqtt message.
/*
void MQTTSender::landMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("I heard a land Signal. Sending it out over MQTT.\n");
  std::string land = "land";
  publish(NULL, "/ardrone/land",  land.length() , (const uint8_t *)land.data());
}

//This function is called when a reset message is received over ROS topic. It publishes out the corresponding mqtt message.
void MQTTSender::resetMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("I heard a reset Signal. Sending it out over MQTT.\n");
  std::string reset = "reset";
  publish(NULL, "/ardrone/reset",  reset.length() , (const uint8_t *)reset.data());
}
*/
//This function is called when a cmd_vel message is received over ROS topic. It publishes out the corresponding mqtt message.
//This message is usually sent by the tum_ardrone.
void MQTTSender::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  //Code taken from CmdVelCallback in src/teleop_twist.cpp in the ardrone_autonomy package 
  //Code also taken from updateTeleop function in src/teleop_twist.cpp

  
/*  //Check the bounds of the values are between -1 and 1. All other are out-of-range values.
  float left_right = static_cast<float>(std::max(std::min(-msg->linear.y, 1.0), -1.0));
  float front_back = static_cast<float>(std::max(std::min(-msg->linear.x, 1.0), -1.0));
  float up_down = static_cast<float>(std::max(std::min(msg->linear.z, 1.0), -1.0));
  float turn = static_cast<float>(std::max(std::min(-msg->angular.z, 1.0), -1.0));

  //Check if commands have changed from the previous commands. 
  bool is_changed = !(
      (fabs(left_right - old_left_right) < _EPS) &&
      (fabs(front_back - old_front_back) < _EPS) &&
      (fabs(up_down - old_up_down) < _EPS) &&
      (fabs(turn - old_turn) < _EPS));

  // These lines are for testing, they should be moved to configurations
  // Bit 0 of control_flag == 0: should we hover?
  // Bit 1 of control_flag == 1: should we use combined yaw mode?

  int32_t control_flag = 0x00;
  int32_t combined_yaw = 0x00;

  // Auto hover detection based on ~0 values for 4DOF cmd_vel
  int32_t hover = (int32_t)
    (
     (fabs(left_right) < _EPS) &&
     (fabs(front_back) < _EPS) &&
     (fabs(up_down) < _EPS) &&
     (fabs(turn) < _EPS) &&
     // Set angular.x or angular.y to a non-zero value to disable entering hover
     // even when 4DOF control command is ~0
     (fabs(msg->angular.x) < _EPS) &&
     (fabs(msg->angular.y) < _EPS));

  control_flag |= ((1 - hover) << 0);
  control_flag |= (combined_yaw << 1);
  // ROS_INFO (">>> Control Flag: %d", control_flag);

  old_left_right = left_right;
  old_front_back = front_back;
  old_up_down = up_down;
  old_turn = turn;
  // is_changed = true;
  if ((is_changed) || (hover))
  {
    //If commands have changed or we are hovering, package data in a binn format and send it over MQTT
*/
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
    ROS_INFO("I heard a cmd_vel Signal size %d. Sending it out over MQTT.\n", binn_size(obj));

    //free the binn object
    binn_free(obj);
 // }
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
  
  std::string takeOffMsgTopic = "/ardrone/takeoff";
  std::string landMsgTopic = "/ardrone/land";
  std::string resetMsgTopic = "/ardrone/reset";
  std::string cmdVelMsgTopic = "/cmd_vel";
	std::string imageMsgTopic = "/ardrone/image_raw";
	std::string navdataMsgTopic = "/ardrone/navdata";
  int brokerPort = 1883;

  //Read the variables from the parameter launch file. If the variable is not mentioned
  //in the parameter launch file, the defaults defined above are used. 
  nodeHandle.getParam("/mqttSender/takeOffMsgTopic", takeOffMsgTopic);
  nodeHandle.getParam("/mqttSender/landMsgTopic", landMsgTopic);
  nodeHandle.getParam("/mqttSender/resetMsgTopic", resetMsgTopic);
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


  //Here, we are subscribing to 4 ROS topics. These topics are published by tum_ardrone. Each topic is handled
  //by a separate callback which is the 3rd argument of the function calls below. On receipt of a message
  //the appropriate callback is called.
  //ros::Subscriber takeOffSub = nodeHandle.subscribe(takeOffMsgTopic, 1000, &MQTTSender::takeOffMessageCallback, mqttSender);
  //ros::Subscriber landSub = nodeHandle.subscribe(landMsgTopic, 1000, &MQTTSender::landMessageCallback, mqttSender);
  //ros::Subscriber resetSub = nodeHandle.subscribe(resetMsgTopic, 1000, &MQTTSender::resetMessageCallback, mqttSender);
  ros::Subscriber cmd_vel_sub = nodeHandle.subscribe(cmdVelMsgTopic, 1000, &MQTTSender::CmdVelCallback, mqttSender);
	ros::Subscriber image_sub = nodeHandle.subscribe(imageMsgTopic, 1000, &MQTTSender::imageMessageCallback, mqttSender);
  ros::Subscriber navdata_sub = nodeHandle.subscribe(navdataMsgTopic, 1000, &MQTTSender::navdataMessageCallback, mqttSender);
 
  /*****/
  //Get the variable from the parameter launch file whether or not to ouput delays to a file
  //If so, then setup the delay files and let the MQTTSender know that it needs to print values
  //to the files.
  bool delayFiles = false;
  nodeHandle.getParam("/mqttSender/outputDelayFiles", delayFiles);
  std::cout << "\nOutputDealyFiles set to " << delayFiles << std::endl;
  if(delayFiles)
  {
    //Setting this to true tells the mqttSender class instance that whenever new navdata and image message is received, 
    //delay values need to be printed to those files.
    mqttSender->outputDelayFiles = true;

    //This function sets up the output files and writes the headers to them.
    mqttSender->setupDelayFiles();
  }
  /******/

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

