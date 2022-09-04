#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <iostream>
#include <math.h>
#include <string>
#include <termios.h>

using namespace std;
int i = 0;
geometry_msgs::Twist speed;

void circle(ros::Publisher velocity_pub);
void eight(ros::Publisher velocity_pub);
void s_shaped(ros::Publisher velocity_pub);
void zigzag(ros::Publisher velocity_pub);
void stop(ros::Publisher velocity_pub);

char buff = 0;

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;

    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old = {0};
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if (rv == -1)
        ROS_ERROR("select");
    else if (rv == 0)
        ROS_INFO("no_key_pressed");
    else
        read(filedesc, &buff, len);

    old.c_cflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR("tcsetattr ~ICANON");
    return (buff);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotont_driver");
    ros::NodeHandle n;
    ros::Publisher velocity_pub;

    velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ros::Rate loop_rate(2);

    ROS_INFO("......START.....");

    cout << "Press o for circle" << endl;
    cout << "Press s for s_shaped" << endl;
    cout << "Press 8 for 8_shaped" << endl;
    cout << "Press z or n for zigzag" << endl;
    cout << "Press t to stop" << endl;

    while (ros::ok())
    {
        char c = 0; // call your non-blocking input function
        c = getch();
        ROS_INFO("%c", c);

        if (c == 'o')
        {
            circle(velocity_pub);
        }

        else if (c == 's')
        {
            s_shaped(velocity_pub);
        }

        else if (c == '8')
        {
            eight(velocity_pub);
        }

        else if (c == 'z' || c == 'n')
        {
            zigzag(velocity_pub);
        }

        else if (c == 't')
        {
            stop(velocity_pub);
        }

        loop_rate.sleep();
    }

    return 0;
}

void circle(ros::Publisher velocity_pub)
{
    speed.linear.x = 1;
    speed.angular.z = 1.5;

    velocity_pub.publish(speed);
}

void eight(ros::Publisher velocity_pub)
{

    i++;

    if (i < 9)
    {
        speed.linear.x = 1;
        speed.angular.z = 1.5;
        velocity_pub.publish(speed);
    }

    if (i >= 9)
    {
        speed.linear.x = 1;
        speed.angular.z = -1.5;
        velocity_pub.publish(speed);
    }

    if (i > 15)
    {
        i = 0;
    }
}

void s_shaped(ros::Publisher velocity_pub)
{

    i++;

    if (i < 5)
    {
        speed.linear.x = 1;
        speed.angular.z = 2;
        velocity_pub.publish(speed);
    }

    if (i >= 5)
    {
        speed.linear.x = 1;
        speed.angular.z = -2;
        velocity_pub.publish(speed);
    }

    if (i > 7)
    {
        i = 0;
        ros::Duration(1).sleep();
    }
}

void zigzag(ros::Publisher velocity_pub)
{

    speed.linear.x = 0.4;
    speed.angular.z = 0;
    velocity_pub.publish(speed);
    ros::Duration(1).sleep();

    speed.linear.x = 0;
    speed.angular.z = 4;
    velocity_pub.publish(speed);
    ros::Duration(1).sleep();

    speed.linear.x = 0.4;
    speed.angular.z = 0;
    velocity_pub.publish(speed);
    ros::Duration(1).sleep();

    speed.linear.x = 0;
    speed.angular.z = -4;
    velocity_pub.publish(speed);
    ros::Duration(1).sleep();
}

void stop(ros::Publisher velocity_pub)
{
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.linear.z = 0;

    speed.angular.x = 0;
    speed.angular.y = 0;
    speed.angular.z = 0;

    velocity_pub.publish(speed);
}
