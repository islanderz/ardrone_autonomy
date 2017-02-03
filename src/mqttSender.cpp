/*******************************************************************************
   Edited by: Dec 2016 Harshit Sureka <harshit.sureka@gmail.com>
 *
 * The mqtt_bridge node for sending images and navdata from the ardrone_autonomy
 *
 *******************************************************************************/

#include "ros/ros.h"
#include "ros/serialization.h"
#include <mosquittopp.h>
#include <image_transport/image_transport.h>
#include <ardrone_autonomy/Navdata.h>

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

    //This is a callback for receiving a navdata msg over ROS. It is then sent over MQTT.
    void navdataMessageCallback(const ardrone_autonomy::Navdata &msg);

    //This is a callback for receiving an image msg over ROS. It is then sent over MQTT.
    void imageMessageCallback(const sensor_msgs::Image &msg);
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

void MQTTSender::on_message(const struct mosquitto_message *message)
{
}

//Callback when the mosquitto library successfully subscribes to a topic
void MQTTSender::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  ROS_INFO("Subscription succeeded.\n");
}

void MQTTSender::navdataMessageCallback(const ardrone_autonomy::Navdata& msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, msg);
  publish(NULL, "/ardrone/navdata", serial_size, obuffer.get());
 
  return;
}
void MQTTSender::imageMessageCallback(const sensor_msgs::Image &msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(msg);
  boost::shared_array<uint8_t> obuffer(new uint8_t[serial_size]);

  ros::serialization::OStream ostream(obuffer.get(), serial_size);
  ros::serialization::serialize(ostream, msg);

	publish(NULL, "/ardrone/image", serial_size, obuffer.get());

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

	std::string imageMsgTopic = "/ardrone/image_raw";
	std::string navdataMsgTopic = "/ardrone/navdata";
  std::string broker = "localhost";
  int brokerPort = 1883;

  //Read the variables from the parameter launch file. If the variable is not mentioned
  //in the parameter launch file, the defaults defined above are used. 
  nodeHandle.getParam("/mqttSender/imageMsgTopic", imageMsgTopic);
  nodeHandle.getParam("/mqttSender/navdataMsgTopic", navdataMsgTopic);
  ros::param::get("/mqttSender/mqttBroker", broker);
  nodeHandle.getParam("/mqttSender/mqttBrokerPort", brokerPort);

  ROS_INFO("Connecting to %s at %d\n", broker.c_str(), brokerPort);
  
  //Initialize the mqttSender class instance
  class MQTTSender *mqttSender;

  mqttSender->lib_init();

  mqttSender = new MQTTSender(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  ROS_INFO("mqttSender initialized..\n");

	ros::Subscriber image_sub = nodeHandle.subscribe(imageMsgTopic, 1000, &MQTTSender::imageMessageCallback, mqttSender);
  ros::Subscriber navdata_sub = nodeHandle.subscribe(navdataMsgTopic, 1000, &MQTTSender::navdataMessageCallback, mqttSender);
 
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

