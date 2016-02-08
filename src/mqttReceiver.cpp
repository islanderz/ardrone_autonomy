#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MQTTAsync.h"
#include "zlib.h"
#include "async_subscribe.h"
extern "C" {
#include "binn.h"
}


/****************************/

#define CLIENTID    "mqttReceiver"
#define QOS         1
#define TIMEOUT     10000L

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
/*
class MQTTROSBridge
{
  public:
  MQTTROSBridge();
  ~MQTTROSBridge();

  MQTTROSBridge(std::string address)
  {
    MQTTAsync_create(&client, address.c_str(), CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);
    connected = 0;
    finished = 0;
  }
    


  int disc_finished;
  int subscribed;
  int connected;
  int finished;

  MQTTAsync client;

  void setCallbacks()
  {
    MQTTAsync_setCallbacks(client, NULL, connlost, msgarrvd, NULL);
  }

  void connlost(void *context, char *cause)
  {
    MQTTAsync client = (MQTTAsync)context;
    MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
    int rc;

    ROS_INFO("\nConnection lost\n");
    ROS_INFO("     cause: %s\n", cause);

    ROS_INFO("Reconnecting\n");
    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    if ((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
    {
      ROS_INFO("Failed to start connect, return code %d\n", rc);
      finished = 1;
    }
  }


  int msgarrvd(void *context, char *topicName, int topicLen, MQTTAsync_message *message)
  {
    //we have received a compressed image stream and need to decompress it
    if(strcmp(topicName,"image/compressedImageStream") == 0)
    {
      int MAX_MSG_SIZE = 50000;//this value should be updated and stored as a global variable. Or hardcoded if we know the max size.
      uint8_t* imgData;
      unsigned long uncompSize;
      imgData = (uint8_t*)malloc(MAX_MSG_SIZE);
      uncompress(imgData, 
          &uncompSize, 
          (uint8_t*)message->payload, 
          message->payloadlen);

      printf("Received image with %d bytes, uncompressed to %ld bytes\n",message->payloadlen, uncompSize);
      // printf("Message is: %s\n",(char*)imgData);
      MQTTAsync_freeMessage(&message);
      MQTTAsync_free(topicName);
      return 1;
    }
    if(strcmp(topicName,"uas/ardrone1/navdata") == 0)
    {
      binn* obj;

      obj = binn_open(message->payload);

      uint32_t timestamp = binn_object_uint32(obj,(char*)"timestamp");
      uint16_t tag = binn_object_uint16(obj, (char*)"tag");
      uint16_t size = binn_object_uint16(obj, (char*)"size");
      uint32_t ctrl_state = binn_object_uint32(obj, (char*)"ctrl_state");
      uint32_t vbat_flying_percentage = binn_object_uint32(obj, (char*)"vbat_flying_percentage");
      float theta = binn_object_float(obj, (char*)"theta");
      float phi = binn_object_float(obj, (char*)"phi");
      float psi = binn_object_float(obj, (char*)"psi");
      uint32_t altitude = binn_object_uint32(obj, (char*)"altitude");
      float vx = binn_object_float(obj, (char*)"vx");
      float vy = binn_object_float(obj, (char*)"vy");
      float vz = binn_object_float(obj, (char*)"vz");
      uint32_t num_frames = binn_object_uint32(obj, (char*)"num_frames");
      uint32_t detection_camera_type = binn_object_uint32(obj, (char*)"detection_camera_type");

      printf("Received navdata msg with angles: %f %f %f\n", theta, phi, psi);
      printf("Received navdata msg with velocities: %f %f %f\n", vx, vy, vz);
      printf("Received navdata msg with battery/ctrl_state/altd: %d/%d/%d\n", vbat_flying_percentage, ctrl_state, altitude);
      printf("\n");

      //NEED TO PUBLISH ON ROS TOPICS HERE

      binn_free(obj);
      return 1;
    }


    int i;
    char* payloadptr;

    ROS_INFO("Received message of length %d on topic %s\n",message->payloadlen,topicName);

    ROS_INFO("Message arrived\n");
    ROS_INFO("     topic: %s\n", topicName);
    ROS_INFO("   message: ");

    payloadptr = (char*)message->payload;
    for(i=0; i<message->payloadlen; i++)
    {
      putchar(*payloadptr++);
    }
    putchar('\n');
    MQTTAsync_freeMessage(&message);
    MQTTAsync_free(topicName);
    return 1;
  }


  void onDisconnect(void* context, MQTTAsync_successData* response)
  {
    ROS_INFO("Successful disconnection\n");
    disc_finished = 1;
  }


  void onSubscribe(void* context, MQTTAsync_successData* response)
  {
    ROS_INFO("Subscribe succeeded\n");
    subscribed = 1;
  }

  void onSubscribeFailure(void* context, MQTTAsync_failureData* response)
  {
    ROS_INFO("Subscribe failed, rc %d\n", response ? response->code : 0);
    finished = 1;
  }


  void onConnectFailure(void* context, MQTTAsync_failureData* response)
  {
    ROS_INFO("Connect failed, rc %d\n", response ? response->code : 0);
    finished = 1;
  }


  void onConnect(void* context, MQTTAsync_successData* response)
  {
    ROS_INFO("Successful connection\n");
    connected = 1;
  }

};


//volatile MQTTAsync_token deliveredtoken;

*/
/*******************************/

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mqttReceiver");
  ros::NodeHandle n;

  /*********
  std::string aa;
  float amin = 0.0;
  std::vector<std::string> myvec;
  ros::param::get("/mqttReceiver/myparam",aa);
  ros::param::get("/mqttReceiver/altitude_min",amin);
  ros::param::get("/mqttReceiver/listTopics",myvec);
  std::cout << myvec.size() << std::endl;
  std::cout << "string: " << aa << std::endl;
  std::cout << "num: " << amin << std::endl;
  **********/
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  
  std::string broker = "tcp://unmand.io";
  std::string brokerPort = "1884";
  ros::param::get("/mqttReceiver/mqttBroker",broker);
  ros::param::get("/mqttReceiver/mqttBrokerPort",brokerPort);

  std::string address = broker + ":" + brokerPort;

  mqtt::async_client client(address.c_str(), CLIENTID);
  action_listener subListener("Subscription");

  callback cb(client, subListener);
  client.set_callback(cb);

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
  /*

     MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
  MQTTAsync_token token;
  int rc;
  
  
  MQTTROSBridge mqttRosBridge(address);

  ROS_INFO("Connecting to ... %s", address.c_str());

  mqttRosBridge.setCallbacks();

  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = &MQTTROSBridge::onConnect;
  conn_opts.onConnectFailure = &MQTTROSBridge::onConnectFailure;
  conn_opts.context = client;
  if((rc = MQTTAsync_connect(mqttRosBridge.client, &conn_opts)) != MQTTASYNC_SUCCESS)
  {
    ROS_ERROR("Failed to start connect, return code %d\n", rc);
  }
  while(!mqttRosBridge.connected)
  {
    sleep(10);
  }
	*/
/*
  MQTTAsync_responseOptions subs_opts = MQTTAsync_responseOptions_initializer; 
  ROS_INFO("Subscribing to %d topics...", topicsList.size());
  subs_opts.onSuccess = &mqttRosBridge::onSubscribe;
  subs_opts.onFailure = &mqttRosBrige::onSubscribeFailure;
  subs_opts.context = mqttRosBridge.client;

  //subscripe to all topics in the param listTopics
  for(int i = 0; i < topicsList.size(); i++)
  {
    if ((rc = MQTTAsync_subscribe(mqttRosBridge.client, topicsList[i].c_str(), QOS, &subs_opts)) != MQTTASYNC_SUCCESS)
    {
      ROS_INFO("Failed to start subscribe, return code %d\n", rc);
    }
  }
*/

  ROS_INFO("Exited Loop\n");
  ros::spin();

  return 0;
}
