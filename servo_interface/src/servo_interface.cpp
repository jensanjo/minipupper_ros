#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <math.h>
#include <stdio.h>

static double degrees(double rad) {
  return rad * 180 / M_PI;
}

static int angle_to_duty_cycle(double angle) {
  return round(500000 + 1000000 * angle / 90.); 
}

static int SERVO_PINS[] = { 15,14,13,  12,11,10,  9,8,7,  6,5,4 };  // rf lf rb lb
static const size_t NSERVOS = 12;

class ServoInterface {
  // std::vector<double> servo_configs;
  double servo_configs[NSERVOS];
  FILE* servos[NSERVOS];
  ros::Subscriber sub;
  int count;

  public:
    ServoInterface(ros::NodeHandle *nh) {
    sub = nh->subscribe("/joint_group_position_controller/command", 1, &ServoInterface::callback, this);
    const char* keys[NSERVOS] = {
      "rf1_initial_angle",
      "rf2_initial_angle",
      "rf3_initial_angle",
      
      "lf1_initial_angle",
      "lf2_initial_angle",
      "lf3_initial_angle",

      "rb1_initial_angle",
      "rb2_initial_angle",
      "rb3_initial_angle",

      "lb1_initial_angle",
      "lb2_initial_angle",
      "lb3_initial_angle",
    };
    for (int i = 0; i < NSERVOS; i++) {
      double x;
      bool res = nh->getParam(keys[i], x);
      ROS_ASSERT_MSG(res, "Failed to get param %s", keys[i]);
      servo_configs[i] = x;
    }
    for (int i = 0; i < NSERVOS; i++) {
      char path[100];
      int pin = SERVO_PINS[i];
      std::snprintf(path, sizeof(path), "/sys/class/pwm/pwmchip0/pwm%d/duty_cycle", pin);
      FILE *f = fopen(path, "w");
      ROS_ASSERT_MSG(f != NULL, "Failed to open %s", path);
      servos[i] = f;
    }
    count = 0;
  }

  void set_angle(int i, double angle) {
    char buf[100];
    int duty_cycle = angle_to_duty_cycle(angle);
    FILE *servo = servos[i];
    std::fprintf(servo, "%d", duty_cycle);
    std::fflush(servo);
  }

  void callback(const trajectory_msgs::JointTrajectory::ConstPtr& msg) {
    auto positions = msg->points[0].positions;
    double lf1 = degrees(positions[0]);
    double lf2 = degrees(positions[1]);
    double lf3 = degrees(positions[2]);
    double rf1 = degrees(positions[3]);
    double rf2 = degrees(positions[4]);
    double rf3 = degrees(positions[5]);
    double lb1 = degrees(positions[6]);
    double lb2 = degrees(positions[7]);
    double lb3 = degrees(positions[8]);
    double rb1 = degrees(positions[9]);
    double rb2 = degrees(positions[10]);
    double rb3 = degrees(positions[11]);

    double angles[NSERVOS] = {
       rf1,
      -rf2,
      -90 - rf2 - rf3,

      lf1,
      lf2,
      90 + lf2 + lf3,

      -rb1,
      -rb2,
      -90 - rb2 - rb3,

      -lb1,
      lb2,
      90 + lb2 + lb3,
    };
    for (int i = 0; i < NSERVOS; i++) {
      double angle = angles[i] + servo_configs[i];
      set_angle(i, angle);
    }
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "servo_listener"); 
  ros::NodeHandle nh;
  ServoInterface si(&nh);
  ros::spin();
  return 0;
}