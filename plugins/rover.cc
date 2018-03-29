#ifndef ROVER_PLUGIN
#define ROVER_PLUGIN

// Include Gazebo headers.
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

// Include ROS headers so we can communicate with our robot
#include <ros/ros.h>

// Include std::string's because they're pretty darn useful.
#include <string>

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

    public:
    RoverPlugin() {}

    // Runs each nanosecond tick
    void onUpdate(const common::UpdateInfo &inf)
    {
       _m->GetJoint("jFL")->SetForce(0, 5);
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

       // Bind our onUpdate function to the update callback
       this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RoverPlugin::onUpdate, this, _1));
    }
  };

  // Gazebo macro to set up the rest of the plugin functionality
  GZ_REGISTER_MODEL_PLUGIN(RoverPlugin)
}

#endif


