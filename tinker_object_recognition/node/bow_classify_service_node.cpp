#include <ros/ros.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <tinker_vision_msgs/ObjectAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <tinker_vision_msgs/FindObjects.h>

//using namespace tinker::vision;
using std::string;
using std::vector;

typedef actionlib::SimpleActionClient<tinker_vision_msgs::ObjectAction> Client;


class BoWClassifyClientNode {
public:
    BoWClassifyClientNode() 
        :   private_nh_("~"),
            ac_("find_object", true) {
        private_nh_.param("sample_count", sample_count_, 10);
        private_nh_.param("accept_count", accept_count_, 5);
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
    
    bool FindObjectService(
        tinker_vision_msgs::FindObjects::Request &req,
        tinker_vision_msgs::FindObjects::Response &res) {
        tinker_vision_msgs::ObjectGoal goal;
        goal.sample_count = sample_count_;
        goal.accept_count = accept_count_;
        ac_.sendGoal(goal,
                boost::bind(&BoWClassifyClientNode::doneCB, this, _1, _2));
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
