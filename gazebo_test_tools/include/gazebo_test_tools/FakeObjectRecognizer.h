#ifndef GAZEBO_TEST_TOOLS_FAKEOBJECTRECOGNISER_H
#define GAZEBO_TEST_TOOLS_FAKEOBJECTRECOGNISER_H

#include <ros/ros.h>

#include <object_msgs/Object.h>
#include <object_msgs/ObjectInfo.h>
#include <object_msgs/ObjectInfoRequest.h>
#include <object_msgs/ObjectInfoResponse.h>

#include <gazebo_test_tools/RecognizeGazeboObject.h>
#include <gazebo_test_tools/RecognizeGazeboObjectRequest.h>
#include <gazebo_test_tools/RecognizeGazeboObjectResponse.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>
#include <boost/thread/mutex.hpp>

namespace gazebo_test_tools
{

/**
 * This is a helper for *moveit_object/CollisionObjectGenerator*: A object_msgs/Object is
 * *only* published after a service request of type *gazebo_test_tools/RecognizeGazeboObject*
 * is sent to this helper. This avoids *object_msgs/Object*
 * messages *continuosuly* having to be published and instead allows more control about when new
 * object information is available, as it would be with a regular object recognition.
 *
 * Note that the class gazebo_state_plugins/GazeboObjectInfo also offers the functionality
 * to continusously publish object_msgs/Object.msg, but it does so for *all* existing
 * objects (except the ones excluded at start-up). 
 * With the FakeObjectRecognizer, it is possible to switch on/off the publishing
 * of object_msgs/Object information for specific objects only at various times.
 * Because of this, when you use FakeObjectRecogniser to send information to
 * moveit_object/CollisionObjectGenerator, you will need to make sure that
 * gazebo_state_plugins/GazeboObjectInfo is run with this parameter set to false:
 * 
 * ``gazebo_state_plugins/publish_world_objects: false``
 *
 * This is because in this case, you want the FakeObjectRecognizer to be the one to publish
 * the information instead of GazeboObjectInfo.
 *
 * \author Jennifer Buehler
 * \date February 2016
 **/
class FakeObjectRecognizer {
private:

    typedef object_msgs::Object ObjectMsg;
    typedef object_msgs::ObjectInfo ObjectInfoMsg;

public:

    FakeObjectRecognizer();
    virtual ~FakeObjectRecognizer();

private:

    bool recognizeObject(gazebo_test_tools::RecognizeGazeboObject::Request  &req, gazebo_test_tools::RecognizeGazeboObject::Response &res);

    void publishRecognitionEvent(const ros::TimerEvent& e); 

    bool getGazeboObject(const std::string& name, object_msgs::Object& object, bool include_geometry);

    std::string OBJECTS_TOPIC;

    // service name/topic to request information about
    // objects with object_msgs/ObjectInfo.srv
    std::string SERVICE_REQUEST_OBJECT_TOPIC;

    // service which will be offered to recognise
    // an object, of type gazebo_test_tools/TriggerRecognition.srv.
    std::string SERVICE_RECOGNISE_OBJECT_TOPIC;

    // Recognised objects which are to be continuously published
    // as recognised are published at this rate.
    float PUBLISH_RECOGNISED_OBJECT_RATE;

    ros::Publisher object_pub;
    ros::ServiceClient object_info_client;
    
    ros::ServiceServer recognize_object_srv;

    // all objects which were set to be continuously published
    std::set<std::string> addedObjects;
    
    // mutex for addedObjects
    boost::mutex addedObjectsMtx;

    ros::Timer publishTimer;
};

}  // namespace gazebo_test_tools

#endif  // GAZEBO_TEST_TOOLS_FAKEOBJECTRECOGNISER_H