#ifndef EDUMIP_CODY_PLUGIN
#define EDUMIP_CODY_PLUGIN

// Gazebo included, because well this is a gazebo plugin
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// ROS included so we can communicate back and forth
#include <ros/ros.h>

#include <string>

// Used messages
#include "geometry_msgs/Twist.h"

// Figure out if a is positive or negative
#define SIGN(a) (a >= 0 ? 1.0 : -1.0)

// Find the absolute value of a
#define MAG(a) (a * SIGN(a))

// If a is 0, use value b instead
#define NZ(a, b) (a == 0 ? b : a)

// If the magnitude is above b, use b with the sign of a
#define MAX_MAG(a, b) (MAG(a) > b ? b * SIGN(a) : a)

// If the magnitude is below b, use b with the sign of a
#define LES_MAG(a, b) (MAG(a) <= MAG(b) ? a : b)

// Converts from -pi to pi to 0 to 2pi
#define ZTP(a) (a < 0 ? a + 2.0 * PI : a)
// Converts form 0 to 2pi to -pi to pi
#define NPP(a) (a > PI ? 2.0 * PI - a : a)

// This is the maximum effort our motors can put out
// And the max the simulation will use
#define MAX_MOTOR_TORQUE 0.30

// This is the maximum value we will scale our desired effort to
#define EFFORT_MAX_MAG 0.49

// This is the maximum effort we will scale our turning effort to
#define YAW_MAX_EFFORT 0.05

// This is the highest magnitude pitch from standing that we want to
// ever try to obtain (in radians)
#define UPPER_PITCH_BOUND 0.050

// Maximum velocities we will aim for (in units/(LOOPTIME ms))
#define MAX_LIN_V 0.05
#define MAX_YAW_V 0.03
#define MAX_PITCH_V 0.01

// Pi is a useful number to have
#define PI 3.14159265359

// Realtime controllers only update on a set loop, so we don't want
// to pretend that our edumip will be able to update faster than it can
// so we have some logic to restrict the processing speed of our edumip
// simulation.
#define LOOPTIME_MS 20.0

// A lot of the time required is in nanoseconds, but our controller works
// on milisecond times not nanosecond.
#define MS_TO_NS(a) a * 1000000.0

// So we don't have to do gazebo::everything
namespace gazebo
{

  // Defining our plugin
  class WBPlugin : public ModelPlugin
  {
    private:
      // This pointer gives us access to all of the state information
      // of our edumip model
      physics::ModelPtr _m;

      // This pointer gives us access to all of the state information
      // of the simulated world.
      physics::WorldPtr _w;

      // Timestamp of the last processing cycle of our edumip.
      // Used to help us simulate the limited processing on a realtime
      // controller.
      common::Time lastSimTime;

      // Pointer to event callback for our onUpdate function
      // Called every nanosecond of the simulation.
      event::ConnectionPtr updateConnection;

      // Our ROS pointers.
      std::unique_ptr<ros::NodeHandle> node;
      ros::Subscriber *sub;

      // Prior positions and rotations to calculate velocities.
      math::Quaternion pRot;
      math::Vector3 pPos; 

      // Last given velocity and yaw commands.
      double lc_V;
      double lc_Y;
      // Timestamp of our last recieved command.
      common::Time lc_T;
    public:
      WBPlugin() {}

      // Called when we spin() in order to deal with the given geometry
      // messages on the stack.
      void onCmd(geometry_msgs::Twist t)
      {
          // Update our last given commands.
          lc_T = _w->GetSimTime();
          lc_V = t.linear.x;
          lc_Y = t.angular.z;
      }

      // Called every nanosecond of sim-time
      void onUpdate(const common::UpdateInfo &inf)
      { 
         // A real microcontroller doesn't process every nanosecond, 
         // so we're only gonna process after X milliseconds to simulate 
         // the firmware we are gonna work with
         common::Time cSimTime = _w->GetSimTime();
         common::Time wait(0, MS_TO_NS(LOOPTIME_MS));
         common::Time commandTimeout(5, MS_TO_NS(0));
         if (cSimTime - lastSimTime < wait)
         {
           return;
         }
         lastSimTime = cSimTime;

         if (cSimTime - lc_T > commandTimeout)
         {
           lc_T = cSimTime;
           lc_Y = lc_V = 0;
         }

         // Get current rotation and position information of our Edumip
         math::Quaternion rot = _m->GetWorldPose().rot;
         math::Vector3 pos = _m->GetWorldPose().pos;

         // Distance travelled in last timestep
         math::Vector3 dist = pos - pPos;

         // Translate our inputted control into the desired speed.
         double desVel = lc_V * MAX_LIN_V;

         // Current rotations
         double roll = rot.GetRoll();
         double pitch = rot.GetPitch();
         double yaw = rot.GetYaw();

         // Angular velocities
         double yawV = yaw - pRot.GetYaw();
         double pitchV = pitch - pRot.GetPitch();

         // Is our velocity forward or backwards compared to the "front"
         // of the robot (rotate cartesian coordinates by yaw and then
         // see if X is positive)
         double velSign = SIGN(dist.x * cos(yaw) + dist.y * sin(yaw));
         double vel = velSign * sqrt(dist.x * dist.x + dist.y * dist.y);

         // How much we can yaw without falling depends on our velocity.
         double maxYV = NZ(((MAX_LIN_V-MAG(vel))/(MAX_LIN_V))*MAX_YAW_V, 0.00001);

         // The more we pitch the faster we accellerate. So we control velocity
         // with pitch
         double desPitch = ((desVel - vel) / (MAX_LIN_V)) * UPPER_PITCH_BOUND; 

         // Normalize pitch, and then figure out how much correction we need on
         // the wheels in order to get the right pitch
         double normPitchDiff = (desPitch - pitch) / (PI / 3.25);
         double correctionEffort = normPitchDiff * 
            ((SIGN(normPitchDiff) * MAX_PITCH_V - pitchV) / 
                (SIGN(normPitchDiff) * MAX_PITCH_V)) * EFFORT_MAX_MAG;

         // Figure out our yaw control
         double yawDif = lc_Y;

         // Effort on wheels affects yaw accelleration, 
         // so we control that to control yaw velocity.
         double yawCorrection = ((yawDif*maxYV-yawV)/(maxYV))*(YAW_MAX_EFFORT); 

         // Send motor commands
         double rEff = MAX_MAG(correctionEffort-yawCorrection, MAX_MOTOR_TORQUE);
         double lEff = MAX_MAG(correctionEffort+yawCorrection, MAX_MOTOR_TORQUE);
         _m->GetJoint("jointR")->SetForce(0,rEff);
         _m->GetJoint("jointL")->SetForce(0,lEff); 

         // Update prior position and rotation info.
         pRot = rot;
         pPos = pos;

         // Take the commands off of the stack.
         ros::spinOnce();
     }

     // Runs once when the model is loaded
     virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
     {
       _m = _model;

       if(!ros::isInitialized)
       {
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed");
         return;
       }
       ROS_INFO("Wheely Boi Plugin Loaded");

       // For some reason the joint friction isn't loaded from the URDF
       // so we have to manually update them here
       _m->GetJoint("jointR")->SetParam("friction", 0, 0.0);
       _m->GetJoint("jointL")->SetParam("friction", 0, 0.0);

       std::string name = _m->GetName();

       // Spawn our node
       ROS_INFO("Spawning node: %s", name.c_str());
       node.reset(new ros::NodeHandle(name.c_str())); 
       ROS_INFO("Node spawned.");

       // Initialize our ROS subscriber
       sub = new ros::Subscriber();
       std::string subName = name + "/cmd";
       *sub = node->subscribe(subName, 1, &WBPlugin::onCmd, this);

       _w = _m->GetWorld();
       lastSimTime = _w->GetSimTime();

       lc_V = lc_Y = 0;
       lc_T = lastSimTime;

       // Initialize our prior positions and rotations.
       pRot = _m->GetWorldPose().rot;
       pPos = _m->GetWorldPose().pos;

       // Bind our onUpdate function to a callback that happens every nanosecond
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&WBPlugin::onUpdate, this, _1));
      }
  };

  // Does all of the work to set up the rest of the plugin functionality.
  GZ_REGISTER_MODEL_PLUGIN(WBPlugin)
}

#endif


