#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MQTTAsync.h"
#include <image_transport/image_transport.h>
#include <sys/time.h>
#include <fstream>
#include <ardrone_autonomy/Navdata.h>
#include "zlib.h"
extern "C" {
#include "binn.h"
}


/****************************/

std::string CLIENTID("mqttReceiveer");

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
  image_transport::ImageTransport it_;
  ros::Publisher navdataPub_;
  image_transport::Publisher imagePub_;

  long long int diff_ms;
  int navdataCount;
  std::fstream navdataFile;


//  ros::Publisher imagePub_;

	void reconnect() {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		mqtt::connect_options connOpts;
		connOpts.set_keep_alive_interval(20);
		//connOpts.set_clean_session(true);

	  cli_.connect(connOpts, nullptr, *this);
  }

	// Re-connection failure
	virtual void on_failure(const mqtt::itoken& tok) {
		std::cout << "Reconnection failed. Trying to reconnect.." << std::endl;
		reconnect();
	}

  // Re-connection success
  virtual void on_success(const mqtt::itoken& tok) {
    std::cout << "Reconnection success" << std::endl;;
    //cli_.subscribe("hello", QOS, nullptr, listener_);
    std::vector<std::string> topicsList;
    ros::param::get("/mqttReceiver/topicsList",topicsList);
    for(int i =0 ; i < topicsList.size(); i++)
    {

      std::cout << "Subscribing to topic " << topicsList[i] << "\n"
        << "for client " << CLIENTID
        << " using QoS" << QOS << "\n\n"
        << "Press Q<Enter> to quit\n" << std::endl;

      cli_.subscribe(topicsList[i].c_str(), QOS, nullptr, listener_);
    }
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
		//std::cout << "Message arrived on topic " << std::endl;
		//std::cout << "\ttopic: '" << topic << "'" << std::endl;

    if(topic == "uas/uav1/compressedImageStream" || topic == "uas/uav1/uncompressedImageStream")
    {
      std::cout << "Received Image msg" << std::endl;
      unsigned long imgDataLen;
      uint8_t* imgData;

      //binn* obj;

      //obj = binn_open((void*)(msg->get_payload()).data());

      sensor_msgs::Image image_msg;
      image_msg.header.stamp = ros::Time::now();//shared_video_receive_time;

      uint32_t imageWidth = 640;//binn_object_uint32(obj,(char*)"width");
      image_msg.width = imageWidth;
      image_msg.height = 360;//binn_object_uint32(obj,(char*)"height");
      image_msg.encoding = "rgb8";
      image_msg.is_bigendian = false;
      //SUREKA-NOTE-TODO Make sure that the step is width*3 and not height*3 ... 
      image_msg.step = imageWidth * 3;

      if(topic == "uas/uav1/compressedImageStream")
      {
        //need to uncompress
        int comp_size = (msg->get_payload()).length();
        //uint8_t* comp_data;
        //void** binn_image_data;
        //= (uint8_t*)(binn_object_blob(obj,(char*)"data", &comp_size));
        //if(binn_object_get_blob(obj, (char*)"data", binn_image_data, &comp_size))
       // {
          //omp_data = (uint8_t*)binn_image_data[0];
          imgData = new uint8_t[5*comp_size];
          int ret_uncp = uncompress(imgData, &imgDataLen, (uint8_t*)(msg->get_payload()).data(), comp_size);
          if (ret_uncp == Z_MEM_ERROR){printf("ERROR: uncompression memory error\n");return;}
          else if (ret_uncp == Z_BUF_ERROR){printf("ERROR: uncompression buffer error\n");return;}
          else if (ret_uncp == Z_DATA_ERROR){printf("ERROR: uncompression data error\n");return;}
          else if (ret_uncp != Z_OK){printf("ERROR: uncompression error unknown\n");return;}
        //}
       // else
       // {
       //   std::cout << "Binn Returned no blob of image data\n";
       //   return;
      //  }
        
        //handle unsuccessful uncompression

        std::cout << "Uncompressed from " << comp_size << " to " << imgDataLen << " bytes" << std::endl;
      } 
      else
      {
        //uncompressed Image
        //std::cout << "hereawefljadlsfkj\n";
        //imgData = (uint8_t*)(binn_object_blob(obj,(char*)"data", (int*)&imgDataLen));
        //int temp_size = 0;
        //void** binn_image_data;
        
        //if(binn_object_get_blob(obj, (char*)"data", binn_image_data, &temp_size))
        //{
          imgData = (uint8_t*)(msg->get_payload()).data();
          imgDataLen = (msg->get_payload()).length();
        //}
        //else
       // {
        //  std::cout << "Binn returned no blob of image data\n";
        //  return;
       // }
      }
      /*********************/
      //image_msg.data.resize(D1_STREAM_WIDTH * D1_STREAM_HEIGHT * 3);
      image_msg.data.resize(imgDataLen);

      //if (!realtime_video) vp_os_mutex_lock(&video_lock);
      //std::copy(buffer, buffer + (D1_STREAM_WIDTH * D1_STREAM_HEIGHT * 3), image_msg.data.begin());
      //std::cout << "here234\n";
      if(imgData != NULL && imgDataLen != 0)
      {
        std::copy(imgData, imgData + imgDataLen, image_msg.data.begin());
        imagePub_.publish(image_msg);
      }
      //if (!realtime_video) vp_os_mutex_unlock(&video_lock);
      //std::cout << "here456\n";

        //hori_pub.publish(image_msg, cinfo_msg_hori);
     /***********************/

      //delete[] imgData;//uncompressed_data;
    }
    else if(topic == "uas/uav1/navdata")
    {
      struct timeval tv;
      gettimeofday(&tv, NULL);

      binn* obj;

      obj = binn_open((void*)(msg->get_payload()).data());
      
      ardrone_autonomy::Navdata navMsg;
      navMsg.header.stamp = ros::Time::now();

      uint32_t org_sec = binn_object_uint32(obj, (char*)"time_sec");
      uint32_t org_usec = binn_object_uint32(obj, (char*)"time_usec");

      uint32_t delaysec = (uint32_t)tv.tv_sec - org_sec;

      double msgTime = (double)(org_sec) + (double)org_usec/1000000.0;
      double recvTime = (double)tv.tv_sec + (double)tv.tv_usec/1000000.0;

      navdataFile << std::fixed << msgTime << ", ";
      navdataFile << std::fixed << recvTime << ", ";
      navdataFile << recvTime - msgTime << std::endl;

      diff_ms = diff_ms + (delaysec)*1000000L + tv.tv_usec - org_usec;  
      navdataCount++;
      if(navdataCount >= 200)
      {
        std::cout << "Average delay of last 200 msgs: " << ((double)diff_ms/1000000L)/200.0 << " sec" << std::endl;
        navdataCount = 0;
        diff_ms = 0;
      }

      //std::cout << org_sec << " " << tv.tv_sec << std::endl;
      //std::cout << "Count: " << navdataCount << std::endl;//" " << "Delay is " << delaysec << " sec and " << delayusec << " ms" << std::endl;

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

      //printf("Received navdata msg with angles: %f %f %f\n", theta, phi, psi);
      //printf("Received navdata msg with velocities: %f %f %f\n", vx, vy, vz);
      //printf("Received navdata msg with battery/ctrl_state/altd: %d/%d/%d\n", vbat_flying_percentage, ctrl_state, altitude);
      //printf("Received a Navdata msg over MQTT. Sending it out as a msg on ROS Topic.\n");

      //NEED TO PUBLISH ON ROS TOPICS HERE

      //conversions from ardrone_driver.cpp


      // positive means counterclockwise rotation around axis


      return;
    }
		//std::cout << "\t'" << msg->to_str() << "'\n" << std::endl;
	}

	virtual void delivery_complete(mqtt::idelivery_token_ptr token) {}

public:
  callback(mqtt::async_client& cli, action_listener& listener, ros::NodeHandle nh) 
    : cli_(cli), listener_(listener), nh_(nh), it_(nh), diff_ms(0), navdataCount(0) {

      struct tm *navdata_atm = NULL;
      struct timeval tv;
      char filename[100];
      gettimeofday(&tv,NULL);
      time_t temptime = (time_t)tv.tv_sec;
      navdata_atm = localtime(&temptime);
      strcpy(filename, "delays");

      sprintf(filename, "%s_%04d%02d%02d_%02d%02d%02d.txt",
          filename,navdata_atm->tm_year+1900, navdata_atm->tm_mon+1, navdata_atm->tm_mday,
          navdata_atm->tm_hour, navdata_atm->tm_min, navdata_atm->tm_sec);


      std::cout << "Writing Navdata msg times and delays to " << filename << std::endl;
      navdataFile.open(filename, std::fstream::out | std::fstream::app);

      navdataFile << "MessageTime(s), ReceiveTime(s), Delay(s)" << std::endl;


  }

  void initPublishers()
  {
    navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("ardrone/navdata", 200);
    imagePub_ = it_.advertise("ardrone/image_raw", 10); 
  }
};

int main(int argc, char **argv)
{
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  
  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle nodeHandle;

  std::string broker = "tcp://unmand.io";
  std::string brokerPort = "1884";
  ros::param::get("/mqttReceiver/mqttBroker",broker);
  ros::param::get("/mqttReceiver/mqttBrokerPort",brokerPort);

  std::string address = broker + ":" + brokerPort;


  mqtt::async_client client(address.c_str(), CLIENTID.c_str());
  action_listener subListener("Subscription");

  callback cb(client, subListener,nodeHandle);
  client.set_callback(cb);

  //initialize the navdata publisher
  cb.initPublishers();
    

  mqtt::connect_options connOpts;
  connOpts.set_keep_alive_interval(20);
  //.connOpts.set_clean_session(true);

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
