#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>

ros::Publisher cmdVelPub;

void shutdown(int sig)
{
  cmdVelPub.publish(geometry_msgs::Twist());//使机器人停止运动
  ROS_INFO("goforward cpp ended!");
  ros::shutdown();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "GoForward");//初始化ROS,它允许ROS通过命令行进行名称重映射
  std::string topic = "/cmd_vel";
  ros::NodeHandle node;//为这个进程的节点创建一个句柄
  cmdVelPub = node.advertise<geometry_msgs::Twist>(topic, 1);//告诉master将要在/cmd_vel topic上发布一个geometry_msgs/Twist的消息
  ros::Rate loopRate(10);//The desired rate to run at in Hz,ros::Rate对象可以允许你指定自循环的频率
  // Override the default ros sigint handler. This must be set after the first NodeHandle is creat
  signal(SIGINT, shutdown);
  ROS_INFO("goforward cpp start...");
  geometry_msgs::Twist speed; // 控制信号载体 Twist message
  while (ros::ok())
  {
    speed.linear.x = 0; // 设置线速度为0m/s，正为前进，负为后退
    speed.angular.z = 0.4; // 设置角速度为0.4rad/s，正为左转，负为右转
    cmdVelPub.publish(speed); // 将刚才设置的指令发送给机器人
    loopRate.sleep();//休眠直到一个频率周期的时间
  }

  return 0;
}
