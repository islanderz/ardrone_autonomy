/*******************************************************************************
   Edited by: Dec 2016 Harshit Sureka <harshit.sureka@gmail.com>
 *
 * Node to send CmdVel, Reset, Takeoff and Land messages over mqtt_bridge
 
 *******************************************************************************/

#include "ros/ros.h"
#include "ros/serialization.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include <mosquittopp.h>

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

   // This is a callback for receiving a cmd_vel message on ROS. It is then sent over MQTT to be received by the sdk.
    void CmdVelCallback(const geometry_msgs::Twist &msg);
    void resetCallback(const std_msgs::EmptyConstPtr);
    void landCallback(const std_msgs::EmptyConstPtr);
    void takeoffCallback(const std_msgs::EmptyConstPtr);
};

//The Constructor
//Intializes Variables, Intializes publishers, Connects the mqtt client.
MQTTSender::MQTTSender(const char *id, const char *host, int port, ros::NodeHandle nh) : 
  mosquittopp(id),
  nh_(nh)
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
  ROS_INFO("Connected with code %d.\n", rc);
}


//When we receive a mqtt message, this callback is called. It just calls the responsible function
//depending on the topic of the mqtt message that was received.
void MQTTSender::on_message(const struct mosquitto_message *message)
{
}

//Callback when the mosquitto library successfully subscribes to a topic
void MQTTSender::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  ROS_INFO("Subscription succeeded.\n");
}

//This function is called when a cmd_vel message is received over ROS topic. It publishes out the corresponding mqtt message.
//This message is usually sent by the tum_ardrone.
void MQTTSender::resetCallback(std_msgs::EmptyConstPtr)
{
  uint8_t dummy[2];dummy[0] = '#';dummy[1] = '#';
  publish(NULL, "/ardrone/reset", 1, dummy);
}
void MQTTSender::takeoffCallback(std_msgs::EmptyConstPtr)
{
  uint8_t dummy[2];dummy[0] = '#';dummy[1] = '#';
  publish(NULL, "/ardrone/takeoff", 1, dummy);
}
void MQTTSender::landCallback(std_msgs::EmptyConstPtr)
{
  uint8_t dummy[2];dummy[0] = '#';dummy[1] = '#';
  publish(NULL, "/ardrone/land", 1, dummy);
}
void MQTTSender::CmdVelCallback(const geometry_msgs::Twist &msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, msg);
  publish(NULL, "/ardrone/cmd_vel", serial_size, obuffer.get());
	
  return;
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
  int brokerPort = 1883;

  //Read the variables from the parameter launch file. If the variable is not mentioned
  //in the parameter launch file, the defaults defined above are used. 
  nodeHandle.getParam("/mqttSender/cmdVelMsgTopic", cmdVelMsgTopic);
  nodeHandle.getParam("/mqttSender/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttSender/mqttBroker", broker);

  ROS_INFO("Connecting to %s at %d\n", broker.c_str(), brokerPort);
  
  //Initialize the mqttSender class instance
  class MQTTSender *mqttSender;

  mqttSender->lib_init();

  mqttSender = new MQTTSender(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  ROS_INFO("mqttSender initialized..\n");

  ros::Subscriber cmd_vel_sub = nodeHandle.subscribe(cmdVelMsgTopic, 1000, &MQTTSender::CmdVelCallback, mqttSender);
  ros::Subscriber reset_sub = nodeHandle.subscribe("/ardrone/reset", 1000, &MQTTSender::resetCallback, mqttSender);
  ros::Subscriber land_sub = nodeHandle.subscribe("/ardrone/land", 1000, &MQTTSender::landCallback, mqttSender);
  ros::Subscriber takeoff_sub = nodeHandle.subscribe("/ardrone/takeoff", 1000, &MQTTSender::takeoffCallback, mqttSender);
 
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

