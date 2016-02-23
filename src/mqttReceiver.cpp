#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include <mosquittopp.h>
#include <image_transport/image_transport.h>
#include <sys/time.h>
#include <fstream>
#include <ardrone_autonomy/Navdata.h>
#include "zlib.h"
extern "C" {
#include "binn.h"
}


/****************************/

std::string CLIENTID("mqttReceiver");

#define QOS         0
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

/////////////////////////////////////////////////////////////////////////////


class mqtt_bridge : public mosqpp::mosquittopp
{

  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Publisher navdataPub_;
    image_transport::Publisher imagePub_;


    long long int diff_ms;
    long long int diff_ms_vid;
    int navdataCount;
    int videoCount;
    std::fstream navdataFile;
    std::fstream videoFile;


  public:
    bool outputDelayFiles;
    mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh);
    ~mqtt_bridge();

    void on_connect(int rc);
    void on_message(const struct mosquitto_message *message);
    void on_subscribe(int mid, int qos_count, const int *granted_qos);
    void setupDelayFiles();
    void initPublishers();
    void handleNavdata(const struct mosquitto_message *message);
    void handleCompressedImage(const struct mosquitto_message *message);
    void handleUncompressedImage(const struct mosquitto_message *message);
    void takeOffMessageCallback(const std_msgs::Empty &msg);
    void landMessageCallback(const std_msgs::Empty &msg);
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

  initPublishers();


  connect_async(host, port, keepalive);
};

mqtt_bridge::~mqtt_bridge()
{
}

void mqtt_bridge::on_connect(int rc)
{
  printf("Connected with code %d.\n", rc);
  if(rc == 0){
    /* Only attempt to subscribe on a successful connect. */
    subscribe(NULL, "temperature/celsius");
  }
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

  uint32_t org_sec; //= binn_object_uint32(obj, (char*)"time_sec");
  uint32_t org_usec;// = binn_object_uint32(obj, (char*)"time_usec");
  memcpy(&org_sec, message->payload, 4);
  memcpy(&org_usec, message->payload + 4, 4);

//  std::cout << "Image recd. with sec: " << org_sec << " and usec: " << org_usec << std::endl;

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
//    std::cout << "Successfully Published Uncompressed Image on Ros topic" << std::endl;
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
  double temp_celsius, temp_farenheit;
  char buf[51];
  if(!strcmp(message->topic, "temperature/celsius")){
    memset(buf, 0, 51*sizeof(char));
    /* Copy N-1 bytes to ensure always 0 terminated. */
    memcpy(buf, message->payload, 50*sizeof(char));
    temp_celsius = atof(buf);
    temp_farenheit = temp_celsius*9.0/5.0 + 32.0;
    snprintf(buf, 50, "%f", temp_farenheit);
    std::cout << "Temperature in F is: " << buf << std::endl;
    publish(NULL, "temperature/farenheit", strlen(buf), buf);
  }
  else if(!strcmp(message->topic, "uas/uav1/navdata"))
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

//  std::cout << "Received Message on topic " << message->topic << " length: " << message->payloadlen << std::endl;
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


int main(int argc, char **argv)
{
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle nodeHandle;

  std::string broker = "tcp://unmand.io";
  std::string takeOffMsgTopic = "/ardrone/takeoff";
  std::string landMsgTopic = "/ardrone/land";
  int brokerPort;
  nodeHandle.getParam("/mqttReceiver/takeOffMsgTopic", takeOffMsgTopic);
  nodeHandle.getParam("/mqttReceiver/landMsgTopic", landMsgTopic);
  nodeHandle.getParam("/mqttReceiver/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttReceiver/mqttBroker",broker);

  //std::string address = broker + ":" + brokerPort;
  std::cout << "Connecting to " << broker << " at " << brokerPort << " port\n";
  
  class mqtt_bridge *mqttBridge;

  mosqpp::lib_init();

  //  mqttBridge = new mqtt_bridge((const char*)"tempconv", (const char*)"localhost", 1883, nodeHandle);
  mqttBridge = new mqtt_bridge(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  std::cout << "mqttBridge initialized..\n";

  ros::Subscriber takeOffSub = nodeHandle.subscribe(takeOffMsgTopic, 1000, &mqtt_bridge::takeOffMessageCallback, mqttBridge);
  ros::Subscriber landSub = nodeHandle.subscribe(landMsgTopic, 1000, &mqtt_bridge::landMessageCallback, mqttBridge);
 
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

