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
class mqtt_bridge : public mosqpp::mosquittopp
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


    //Variables to calculate the delay of a message. 
    long long int diff_ms;//diff in time for navdata
    long long int diff_ms_vid;//diff in time for video
    int navdataCount;//Count of navdata messages. Is reset to 0 after every 200 messages
    int videoCount;//Count of video messages. Is reset to 0 after every 30 messages


    //The file streams for the delay output files for navdata and video
    std::fstream navdataFile;
    std::fstream videoFile;

    //Variables for handling the cmd_vel callback
    
    float old_left_right;
    float old_front_back;
    float old_up_down;
    float old_turn;


  public:
    //Variable to decide if delay files are to be written out.
    bool outputDelayFiles;

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
    void handleCompressedImage(const struct mosquitto_message *message);

    //Callback reidrects here when a uncompressedImage message is received over MQTT. The timestamp is extracted and then 
    //the file is packaged into an imageTransport message and sent as a ROS topic.
    void handleUncompressedImage(const struct mosquitto_message *message);

    //This is a callback for receiving a takeoff message on ROS. It is then sent over MQTT to be received by the sdk.
    void takeOffMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a land message on ROS. It is then sent over MQTT to be received by the sdk.
    void landMessageCallback(const std_msgs::Empty &msg);
    
    //This is a callback for receiving a reset message on ROS. It is then sent over MQTT to be received by the sdk.
    void resetMessageCallback(const std_msgs::Empty &msg);

    //This is a callback for receiving a cmd_vel message on ROS. It is then sent over MQTT to be received by the sdk.
    void CmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
};

void mqtt_bridge::initPublishers()
{
  navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("ardrone/navdata", 200);
  imagePub_ = it_.advertise("ardrone/image_raw", 10); 
}


mqtt_bridge::mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh) : 
  mosquittopp(id),
  nh_(nh), 
  it_(nh)
{
  int keepalive = 60;
  diff_ms = 0;
  diff_ms_vid = 0;
  navdataCount = 0;
  videoCount = 0;
  outputDelayFiles = 0;
    
  old_left_right = -10.0;
  old_front_back = -10.0;
  old_up_down = -10.0;
  old_turn = -10.0;

  initPublishers();

  connect_async(host, port, keepalive);
};

mqtt_bridge::~mqtt_bridge()
{
}

void mqtt_bridge::on_connect(int rc)
{
  printf("Connected with code %d.\n", rc);
}

void mqtt_bridge::handleCompressedImage(const struct mosquitto_message *message)
{
  unsigned long imgDataLen;
  uint8_t* imgData;
  imgData = new uint8_t[20*message->payloadlen];
  int ret_uncp = uncompress(imgData, &imgDataLen, (uint8_t*)message->payload, (unsigned long)message->payloadlen);
  if (ret_uncp == Z_MEM_ERROR){printf("ERROR: uncompression memory error\n");return;}
  else if (ret_uncp == Z_BUF_ERROR){printf("ERROR: uncompression buffer error\n");return;}
  else if (ret_uncp == Z_DATA_ERROR){printf("ERROR: uncompression data error\n");return;}
  else if (ret_uncp != Z_OK){printf("ERROR: uncompression error unknown\n");return;}

  std::cout << "Compressed images are not yet handled properly" << std::endl;

  std::cout << "Uncompressed from " << message->payloadlen << " to " << imgDataLen << " bytes" << std::endl;

  return;
}
void mqtt_bridge::handleUncompressedImage(const struct mosquitto_message *message)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  uint32_t org_sec;
  uint32_t org_usec;
  memcpy(&org_sec, message->payload, 4);
  memcpy(&org_usec, message->payload + 4, 4);

  uint32_t delaysec = (uint32_t)tv.tv_sec - org_sec;

  double msgTime = (double)(org_sec) + (double)org_usec/1000000.0;
  double recvTime = (double)tv.tv_sec + (double)tv.tv_usec/1000000.0;

  videoFile << std::fixed << msgTime << ", ";
  videoFile << std::fixed << recvTime << ", ";
  videoFile << recvTime - msgTime << std::endl;

  diff_ms_vid = diff_ms_vid + (delaysec)*1000000L + tv.tv_usec - org_usec;
  videoCount++;
  if(videoCount >= 30)
  {
    std::cout << "Average delay of last 30 video msgs: " << ((double)diff_ms_vid/1000000L)/30.0 << " sec" << std::endl;
    videoCount = 0;
    diff_ms_vid = 0;
  }

  sensor_msgs::Image image_msg;
  image_msg.header.stamp = ros::Time::now();

  uint32_t imageWidth = 640;
  image_msg.width = imageWidth;
  image_msg.height = 360;
  image_msg.encoding = "rgb8";
  image_msg.is_bigendian = false;
  image_msg.step = imageWidth * 3;

  uint8_t* imgData = (uint8_t*)(message->payload+8);
  unsigned long imgDataLen = message->payloadlen - 8;

  image_msg.data.resize(imgDataLen);

  if(imgData != NULL && imgDataLen != 0)
  {
    std::copy(imgData, imgData + imgDataLen, image_msg.data.begin());
    imagePub_.publish(image_msg);
    return;
  }
}
void mqtt_bridge::handleNavdata(const struct mosquitto_message *message)
{
  struct timeval tv;
  gettimeofday(&tv, NULL);

  binn* obj;

  obj = binn_open(message->payload);

  ardrone_autonomy::Navdata navMsg;
  navMsg.header.stamp = ros::Time::now();

  uint32_t org_sec = binn_object_uint32(obj, (char*)"time_sec");
  uint32_t org_usec = binn_object_uint32(obj, (char*)"time_usec");

  uint32_t delaysec = (uint32_t)tv.tv_sec - org_sec;

  double msgTime = (double)(org_sec) + (double)org_usec/1000000.0;
  double recvTime = (double)tv.tv_sec + (double)tv.tv_usec/1000000.0;

  if(outputDelayFiles)
  {
    navdataFile << std::fixed << msgTime << ", ";
    navdataFile << std::fixed << recvTime << ", ";
    navdataFile << recvTime - msgTime << std::endl;
  }

  diff_ms = diff_ms + (delaysec)*1000000L + tv.tv_usec - org_usec;  
  navdataCount++;
  if(navdataCount >= 200)
  {
    std::cout << "Average delay of last 200 navdata msgs: " << ((double)diff_ms/1000000L)/200.0 << " sec" << std::endl;
    navdataCount = 0;
    diff_ms = 0;
  }

  navMsg.batteryPercent = binn_object_uint32(obj, (char*)"vbat_flying_percentage");
  uint32_t ctrl_state = binn_object_uint32(obj, (char*)"ctrl_state");
  navMsg.state = ctrl_state >> 16;

  int32_t pressure = binn_object_uint32(obj, (char*)"pressure");
  navMsg.rotX = (binn_object_float(obj, (char*)"phi")) / 1000.0;
  navMsg.rotY = (binn_object_float(obj, (char*)"theta")) / 1000.0;
  navMsg.rotZ = (binn_object_float(obj, (char*)"psi")) / 1000.0;
  navMsg.altd = binn_object_uint32(obj, (char*)"altitude");
  navMsg.vx = binn_object_float(obj, (char*)"vx");
  navMsg.vy = -1*binn_object_float(obj, (char*)"vy");
  navMsg.vz = -1*binn_object_float(obj, (char*)"vz");
  navMsg.motor1 = binn_object_uint32(obj, (char*)"motor1");
  navMsg.motor2 = binn_object_uint32(obj, (char*)"motor2");
  navMsg.motor3 = binn_object_uint32(obj, (char*)"motor3");
  navMsg.motor4 = binn_object_uint32(obj, (char*)"motor4");
  uint32_t tm = binn_object_uint32(obj, (char*)"tm");
  navMsg.tm = (tm & 0x001FFFFF) + (tm >> 21) * 1000000;
  navdataPub_.publish(navMsg);

  binn_free(obj);

  return;
}

void mqtt_bridge::on_message(const struct mosquitto_message *message)
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

void mqtt_bridge::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  printf("Subscription succeeded.\n");
}

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

void mqtt_bridge::takeOffMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("I heard a takeoff Signal. Sending it out over MQTT.\n");
  std::string takeOff = "takeoff1";
  publish(NULL, "/ardrone/takeoff",  takeOff.length() , (const void *)takeOff.data());
}

void mqtt_bridge::landMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("I heard a land Signal. Sending it out over MQTT.\n");
  std::string land = "land";
  publish(NULL, "/ardrone/land",  land.length() , (const void *)land.data());
}

void mqtt_bridge::resetMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("I heard a reset Signal. Sending it out over MQTT.\n");
  std::string reset = "reset";
  publish(NULL, "/ardrone/reset",  reset.length() , (const void *)reset.data());
}

void mqtt_bridge::CmdVelCallback(const geometry_msgs::TwistConstPtr &msg)
{
  //Code taken from CmdVelCallback in src/teleop_twist.cpp in the ardrone_autonomy package 
  //Code also taken from updateTeleop function in src/teleop_twist.cpp

  float left_right = static_cast<float>(std::max(std::min(-msg->linear.y, 1.0), -1.0));
  float front_back = static_cast<float>(std::max(std::min(-msg->linear.x, 1.0), -1.0));
  float up_down = static_cast<float>(std::max(std::min(msg->linear.z, 1.0), -1.0));
  float turn = static_cast<float>(std::max(std::min(-msg->angular.z, 1.0), -1.0));

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
    // ardrone_tool_set_progressive_cmd(control_flag, left_right, front_back, up_down, turn, 0.0, 0.0);

    binn* obj;
    obj = binn_object();

    binn_object_set_int32(obj, "control_flag", control_flag);
    binn_object_set_float(obj, "left_right", left_right);
    binn_object_set_float(obj, "front_back", front_back);
    binn_object_set_float(obj, "up_down", up_down);
    binn_object_set_float(obj, "turn", turn);

    publish(NULL, "/ardrone/cmd_vel", binn_size(obj), binn_ptr(obj));

    binn_free(obj);
  }
}


int main(int argc, char **argv)
{
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle nodeHandle;

  std::string broker = "tcp://unmand.io";
  std::string takeOffMsgTopic = "/ardrone/takeoff";
  std::string landMsgTopic = "/ardrone/land";
  std::string resetMsgTopic = "/ardrone/reset";
  int brokerPort;
  nodeHandle.getParam("/mqttReceiver/takeOffMsgTopic", takeOffMsgTopic);
  nodeHandle.getParam("/mqttReceiver/landMsgTopic", landMsgTopic);
  nodeHandle.getParam("/mqttReceiver/resetMsgTopic", resetMsgTopic);
  nodeHandle.getParam("/mqttReceiver/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttReceiver/mqttBroker",broker);

  std::cout << "Connecting to " << broker << " at " << brokerPort << " port\n";
  
  class mqtt_bridge *mqttBridge;

  mosqpp::lib_init();

  mqttBridge = new mqtt_bridge(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  std::cout << "mqttBridge initialized..\n";

  ros::Subscriber takeOffSub = nodeHandle.subscribe(takeOffMsgTopic, 1000, &mqtt_bridge::takeOffMessageCallback, mqttBridge);
  ros::Subscriber landSub = nodeHandle.subscribe(landMsgTopic, 1000, &mqtt_bridge::landMessageCallback, mqttBridge);
  ros::Subscriber resetSub = nodeHandle.subscribe(resetMsgTopic, 1000, &mqtt_bridge::resetMessageCallback, mqttBridge);
  ros::Subscriber cmd_vel_sub = nodeHandle.subscribe("/cmd_vel", 1, &mqtt_bridge::CmdVelCallback, mqttBridge);
 
  bool delayFiles = false;
  nodeHandle.getParam("/mqttReceiver/outputDelayFiles", delayFiles);

  std::cout << "\nOutputDealyFiles set to " << delayFiles << std::endl;
  
  if(delayFiles)
  {
    mqttBridge->outputDelayFiles = true;
    mqttBridge->setupDelayFiles();
  }

  std::vector<std::string> topicsList;
  ros::param::get("/mqttReceiver/topicsList",topicsList);
  for(int i =0 ; i < topicsList.size(); i++)
  {
    std::cout << "Subscribing to topic " << topicsList[i] << "\n"
      << "for client " << CLIENTID
      << " using QoS" << QOS << "\n\n";

    mqttBridge->subscribe(NULL, topicsList[i].c_str());
  }
  ros::Rate loop_rate(10);


  int rc;
  while(ros::ok()){
    ros::spinOnce();
    rc = mqttBridge->loop();
    if(rc){
      mqttBridge->reconnect_async();
    }
  }

  ROS_INFO("Disconnecting MQTT....\n");
  mqttBridge->disconnect();

  mosqpp::lib_cleanup();

  return 0;
}

