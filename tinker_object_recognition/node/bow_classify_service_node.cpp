#include <ros/ros.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <tinker_vision_msgs/ObjectAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tinker_vision_msgs/FindObjects.h>
#include <sensor_msgs/Image.h>

//using namespace tinker::vision;
using std::string;
using std::vector;

typedef actionlib::SimpleActionClient<tinker_vision_msgs::ObjectAction> Client;


class BoWClassifyClientNode {
public:
    BoWClassifyClientNode() 
        :   private_nh_("~"),
            ac_("arm_find_objects", true) {
        XmlRpc::XmlRpcValue frame_acceptance;
        private_nh_.getParam("frame_acceptance", frame_acceptance);
        ROS_ASSERT(frame_acceptance.size() > 0);
        ROS_ASSERT(frame_acceptance.hasMember("sample_count"));
        ROS_ASSERT(frame_acceptance["sample_count"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        ROS_ASSERT(frame_acceptance.hasMember("accept_count"));
        ROS_ASSERT(frame_acceptance["accept_count"].getType() == XmlRpc::XmlRpcValue::TypeInt);
        sample_count_ = (int)frame_acceptance["sample_count"];
        accept_count_ = (int)frame_acceptance["accept_count"];
        ROS_INFO("Waiting for action server to start.");
        ac_.waitForServer();
        ROS_INFO("Action server started, sending goal.");
        find_object_server_ = nh_.advertiseService(
            "arm_find_objects", &BoWClassifyClientNode::FindObjectService, this);
    }
    
    void doneCB(const actionlib::SimpleClientGoalState& state,
              const tinker_vision_msgs::ObjectResultConstPtr& result)
    {
        res_.success = result->success;
        res_.objects = result->objects;
        if(res_.success)
        {
            ROS_INFO("found %s", res_.objects.objects[0].type.key.c_str());
        }
        else
        {
            ROS_INFO("found nothing");
        }
    }
    
    void activeCB()
    {
      ;
    }

    // Called every time feedback is received for the goal
    void feedbackCB(const tinker_vision_msgs::ObjectFeedbackConstPtr& feedback)
    {
    }
    
    bool FindObjectService(
        tinker_vision_msgs::FindObjects::Request &req,
        tinker_vision_msgs::FindObjects::Response &res) {
        tinker_vision_msgs::ObjectGoal goal;
        goal.sample_count = sample_count_;
        goal.accept_count = accept_count_;
        ac_.sendGoal(goal,
                boost::bind(&BoWClassifyClientNode::doneCB, this, _1, _2),
                boost::bind(&BoWClassifyClientNode::activeCB, this),
                boost::bind(&BoWClassifyClientNode::feedbackCB, this, _1));
        bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));
        if (finished_before_timeout)
        {
            actionlib::SimpleClientGoalState state = ac_.getState();
            ROS_INFO("Find Object Action finished: %s",state.toString().c_str());
        }
        else
        {
            ROS_INFO("Find Object Action did not finish before the time out.");
        }
        res.success = res_.success;
        res.objects = res_.objects;
    }
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::ServiceServer find_object_server_;
    Client ac_;
    int sample_count_;
    int accept_count_;
    tinker_vision_msgs::FindObjects::Response res_;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "arm_bow_classify");
    BoWClassifyClientNode n;
    ros::spin();
    return 0;
}
