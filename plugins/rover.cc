#ifndef ROVER_PLUGIN
#define ROVER_PLUGIN

// Include Gazebo headers.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Include ROS headers so we can communicate with our robot
#include <ros/ros.h>

// Include std::string's because they're pretty darn useful.
#include <string>

#include <cmath>

#define PI 3.14159265

#define SGN(a) (a >= 0 ? 1.0 : -1.0)

// So we don't have to do gazebo::everything
namespace gazebo
{
  // Defining our plugin class
  class RoverPlugin : public ModelPlugin
  {
    private:
    // Model pointer
    physics::ModelPtr _m;

    // Pointer to our onUpdate callback
    event::ConnectionPtr updateConnection;

    double zeroToTwoPi(double angle)
    {
       double out = std::fmod(angle, 2.0 * PI);
       if (out < 0)
         out = 2.0 * PI + out;

       return out;
    }

    double minAngleAdjust(double a1, double a2)
    {
      return std::atan2(std::sin(a2 - a1), std::cos(a2 - a1));
    }

    double pLA, pRA;

    public:
    RoverPlugin() {}

    // Runs each nanosecond tick
    void onUpdate(const common::UpdateInfo &inf)
    {
       double angleSetpoint = PI / 3.0;
       double lAngle = zeroToTwoPi(_m->GetJoint("lDifJoint")->GetAngle(0).Radian());
       double rAngle = zeroToTwoPi(_m->GetJoint("rDifJoint")->GetAngle(0).Radian());

       double lAV = minAngleAdjust(pLA, lAngle);
       double rAV = minAngleAdjust(pRA, rAngle);

       double lDif = minAngleAdjust(lAngle, angleSetpoint);
       double rDif = minAngleAdjust(rAngle, angleSetpoint);

       lDif = (lDif * SGN(lDif) < PI / 8.0 ? 0 : lDif);
       rDif = (rDif * SGN(rDif) < PI / 8.0 ? 0 : rDif);
 
       double lAd = (lDif)/(PI);
       double rAd = (rDif)/(PI);
       _m->GetJoint("lDifJoint")->SetForce(0, lAd);
       _m->GetJoint("rDifJoint")->SetForce(0, rAd);

       ROS_INFO("R: %f | L: %f", rAd, lAd);

       _m->GetJoint("jFL")->SetForce(0, -1);
       _m->GetJoint("jFR")->SetForce(0, -1); 

       pLA = lAngle;
       pRA = rAngle;
    }

    // Runs when the model is loaded
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
       if(!ros::isInitialized)
       {
         ROS_FATAL_STREAM("ROS node for Gazebo not established. Plugin failed.");
         return;
       }
       ROS_INFO("Rover Plugin Loaded");

       _m = _model;

       pLA = pRA = 0;

       // Bind our onUpdate function to the update callback
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RoverPlugin::onUpdate, this, _1));
    }
  };

  // Gazebo macro to set up the rest of the plugin functionality
  GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}

#endif


