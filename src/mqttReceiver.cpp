#include "ros/ros.h"
#include "std_msgs/String.h"
#include "MQTTAsync.h"
#include "zlib.h"
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


//volatile MQTTAsync_token deliveredtoken;

int disc_finished = 0;
int subscribed = 0;
int connected = 0;
int finished = 0;

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

    uint32_t timestamp = binn_object_uint32(obj,"timestamp");
    uint16_t tag = binn_object_uint16(obj, "tag");
    uint16_t size = binn_object_uint16(obj, "size");
    uint32_t ctrl_state = binn_object_uint32(obj, "ctrl_state");
    uint32_t vbat_flying_percentage = binn_object_uint32(obj, "vbat_flying_percentage");
    float theta = binn_object_float(obj, "theta");
    float phi = binn_object_float(obj, "phi");
    float psi = binn_object_float(obj, "psi");
    uint32_t altitude = binn_object_uint32(obj, "altitude");
    float vx = binn_object_float(obj, "vx");
    float vy = binn_object_float(obj, "vy");
    float vz = binn_object_float(obj, "vz");
    uint32_t num_frames = binn_object_uint32(obj, "num_frames");
    uint32_t detection_camera_type = binn_object_uint32(obj, "detection_camera_type");

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
	MQTTAsync client = (MQTTAsync)context;
	MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
	int rc;

	ROS_INFO("Successful connection\n");
  connected = 1;


}

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

  MQTTAsync client;
  MQTTAsync_connectOptions conn_opts = MQTTAsync_connectOptions_initializer;
  MQTTAsync_disconnectOptions disc_opts = MQTTAsync_disconnectOptions_initializer;
  MQTTAsync_message pubmsg = MQTTAsync_message_initializer;
  MQTTAsync_token token;
  int rc;
  int ch;
  
  std::string broker = "tcp://unmand.io";
  std::string brokerPort = "1884";
  ros::param::get("/mqttReceiver/mqttBroker",broker);
  ros::param::get("/mqttReceiver/mqttBrokerPort",brokerPort);

  std::string address = broker + ":" + brokerPort;

  ROS_INFO("Connecting to ... %s", address.c_str());
  
  MQTTAsync_create(&client, address.c_str(), CLIENTID, MQTTCLIENT_PERSISTENCE_NONE, NULL);

  MQTTAsync_setCallbacks(client, NULL, connlost, msgarrvd, NULL);

  conn_opts.keepAliveInterval = 20;
  conn_opts.cleansession = 1;
  conn_opts.onSuccess = onConnect;
  conn_opts.onFailure = onConnectFailure;
  conn_opts.context = client;
  if((rc = MQTTAsync_connect(client, &conn_opts)) != MQTTASYNC_SUCCESS)
  {
    ROS_ERROR("Failed to start connect, return code %d\n", rc);
 //   exit(-1);	
  }
  while(!connected)
  {
    sleep(10);
  }
	
  std::vector<std::string> topicsList;
  ros::param::get("/mqttReceiver/topicsList",topicsList);

 // char topic1[] = "navdata/altd";
 // char topic2[] = "navdata/orientation";
 // char topic3[] = "image/compressedImageStream";//image/imagestream";
  

	MQTTAsync_responseOptions subs_opts = MQTTAsync_responseOptions_initializer;
	ROS_INFO("Subscribing to %d topics...", topicsList.size());
	subs_opts.onSuccess = onSubscribe;
	subs_opts.onFailure = onSubscribeFailure;
	subs_opts.context = client;

  //subscripe to all topics in the param listTopics
  for(int i = 0; i < topicsList.size(); i++)
  {
    if ((rc = MQTTAsync_subscribe(client, topicsList[i].c_str(), QOS, &subs_opts)) != MQTTASYNC_SUCCESS)
    {
      ROS_INFO("Failed to start subscribe, return code %d\n", rc);
    }
  }

  ros::Rate loop_rate(10);

 // int count = 0;

  while(ros::ok())
  {
  //  ROS_INFO("Inside loop. All seems to be good.\n");
    loop_rate.sleep();
  //  ++count;
  }

  ROS_INFO("Exited Loop\n");

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
