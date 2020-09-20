



/*
 * rosserial PubSub Example
 * Drives the robot in different direction
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <Loki.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
Loki evilLoki = Loki();

void cmdVelCb( const geometry_msgs::Twist& twist_msg){

  float vx=twist_msg.linear.x;
  float vy=twist_msg.linear.y;
  float w=twist_msg.angular.z;

  if (vx >0){evilLoki.go_advance();}
  else if (vx<0){evilLoki.go_back();}
  else if (w>0){evilLoki.clockwise();}
  else if (w<0){evilLoki.counterclockwise();}
  else {evilLoki.stop_Stop();}


}

// call back function for navigation masage
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmdVelCb );

// Publisher massage 
std_msgs::Int8 sensor_data;
ros::Publisher Sensor_value("Distance_sensor", &sensor_data);
void sendSenorData();

long publisher_timer;

void setup()
{
  nh.initNode();
  nh.advertise(Sensor_value);
  nh.subscribe(sub);
}

void loop()
{
  if (millis() > publisher_timer)
  {
    sendSenorData();
    publisher_timer = millis() + 350; //publish once a second
  }
  
  nh.spinOnce();
  // delay(500);
}

/*
void lokiDriveDirection( const std_msgs::String& directionMsg){
  
  if (strcmp(directionMsg.data, "Forward") == 0)
  {
    evilLoki.go_advance();
    nh.loginfo("Moving Robot Forward");
  }
  else if (strcmp(directionMsg.data, "Backward") == 0) 
  {
    evilLoki.go_back();
    nh.loginfo("Moving Robot Backward");
  }
  else if (strcmp(directionMsg.data, "Left") == 0) 
  {
    evilLoki.counterclockwise();
    nh.loginfo("Moving Robot Left");
  }
  else if (strcmp(directionMsg.data, "Right") == 0)
  {    
    evilLoki.clockwise();
    nh.loginfo("Moving Robot Right");
  }
  else if (strcmp(directionMsg.data, "Stop") == 0) 
  {
    evilLoki.stop_Stop();
    nh.loginfo("Stopping the robot");
  }
  else if (strcmp(directionMsg.data, "R_shift") == 0) 
  {
    evilLoki.right_shift(200,200,200,200);
    nh.loginfo("Shifting right the robot");
  }
  else if (strcmp(directionMsg.data, "L_shift") == 0)
  {
    evilLoki.left_shift(200,150,150,200);
    nh.loginfo("Shifting right the robot");
  }
  
  nh.loginfo("Got new data");
  nh.loginfo(directionMsg.data);
  
}
*/

void sendSenorData()
{

  sensor_data.data = evilLoki.getCurrentDistance();

  Sensor_value.publish(&sensor_data);

}
