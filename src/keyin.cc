#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <string>

#include <termios.h>

#define SIGN(a) (a >= 0 ? 1.0 : -1.0)
#define MAG(a) (a * SIGN(a))
#define UPPER_THRESH(a,b) (a > b ? b : a)
#define LOWER_THRESH(a,b) (a < b ? b : a)
#define MAG_THRESH(a, b) (MAG(a) > b ? b * SIGN(a) : a)
#define PI 3.14

/*
 Function taken from: https://answers.ros.org/question/63491/keyboard-key-pressed/?answer=218817#post-id-218817
*/
char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
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

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0)
		ROS_INFO("no_key_pressed");
	else
		read(filedesc, &buff, len );

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keycmd");

  std::cout << argc;

  std::string target = (argc >= 2 ? argv[1] : "wheely_boi");
  target += "/";
  target += target;
  target += "cmd";

  std::cout << target;

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<geometry_msgs::Twist>(target, 1);
  ros::Rate loop_rate(50);

  geometry_msgs::Twist t;

  while(ros::ok())
  {
    int c = 0;
    c=getch();
    bool send = false;

    if (c == 'w')
    {
      t.linear.x = UPPER_THRESH(t.linear.x + 0.1, 1.0);
    }
    else if (c == 'a')
    {
      t.angular.z = UPPER_THRESH(t.angular.z + 0.1, 1.0);
    }
    else if (c == 'd')
    {
      t.angular.z = LOWER_THRESH(t.angular.z - 0.1, -1.0);
    }
    else if (c == 's')
    {
      t.linear.x = LOWER_THRESH(t.linear.x - 0.1, -1.0);
    }
    else if (c == 'q')
    {
      t.linear.x = 0;
      t.angular.z = 0;
    }

    ROS_INFO("Sending Command: v: %f, y: %f", t.linear.x, t.angular.z);
    pub.publish(t);

    loop_rate.sleep();
  }
}

