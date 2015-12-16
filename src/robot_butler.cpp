//Assignment 8 where a point cloud is used to wipe a table
//baxter_traj_streamer will handle motion for the baxter sim
//cwru_pcl_utils will provide point cloud processing to find motion targets
//written by Matt Dobrowsky

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <geometry_msgs/Pose.h>

//arm motion action messages
#include <cwru_action/cwru_baxter_cart_moveAction.h>

class RobotButler {
private:
	ros::NodeHandle nh_;

	CwruPclUtils cwru_pcl_utils_;
	tf::TransformListener tf_listener_;
	tf::StampedTransform transform_;
	Eigen::Affine3f eigen_tf_;

	Baxter_traj_streamer bax_streamer_;

	//table positioning values
	Eigen::Vector3f table_center_;
	Eigen::Vector3f table_norm_;
	Eigen::Vector3f table_major_axis_;
	Eigen::Vector3f table_centroid_;

	//arm to table destination values
	Eigen::Vector3d xdir_;
	Eigen::Vector3d ydir_;
	Eigen::Vector3d zdir_;
	Eigen::Vector3d origin_;

	//Matrix, Affine, and Pose for arm motions
	Eigen::Matrix3d goal_matrix_;
	Eigen::Affine3d affine_target_;
	geometry_msgs::PoseStamped pose_target_;

	//goals and client needed to carry out arm movement
	cwru_action::cwru_baxter_cart_moveGoal cart_goal_;
    cwru_action::cwru_baxter_cart_moveResult cart_result_;  
	actionlib::SimpleActionClient<cwru_action::cwru_baxter_cart_moveAction> arm_ac_;
	double computed_arrival_time_;

	//suite of private methods for arm operation
	void move_arm_to_goal_pose_(geometry_msgs::PoseStamped pose);
	void displace_arm_by_xyz_(Eigen::Vector3f move);
	void move_arm_to_pre_pose_();
	void execute_arm_move_();

	//helper function to change affine to pose
	geometry_msgs::Pose transform_Affine_to_Pose_(Eigen::Affine3d affine);

public:
	RobotButler(ros::NodeHandle nh);

	~RobotButler(void) { }

	//find for selected points to interpret table location
	//returns true if points were selected
	bool find_table();

	//prepare target pose
	void prepare_motion();

	//executes pose motion
	//make displacement at goal position to "clean"
	void clean_table();	
};

//constructor
RobotButler::RobotButler(ros::NodeHandle nh) : nh_(nh), cwru_pcl_utils_(&nh),
	bax_streamer_(&nh), arm_ac_("cartMoveActionServer", true)
{
	ROS_INFO("Constructing robot butler.");
	//reset selected pcl flag
	cwru_pcl_utils_.reset_got_selected_points();

	//ensure connection to action server and client is good
	ROS_INFO("Waiting for cartesian action server: ");
    bool server_exists = false;
    while ((!server_exists)&&(ros::ok())) {
        server_exists = arm_ac_.waitForServer(ros::Duration(0.5));
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("retrying...");
    }
    ROS_INFO("Connected to arm motion action server.");
}

//public method
//returns true when a group of pcl points have been selected as the "table"
//returns false when no points selected
bool RobotButler::find_table() {

	bool table_found = false;
	ROS_INFO("Resetting Kinect selection flag - table not yet found.");
	//if we find selected points, prepare table data
	if(cwru_pcl_utils_.got_selected_points()) {
		ROS_INFO("Found a new selection of kinect points.");
		table_found = true;
	}
	//else, no points selected, return to loop here again in main
	else {
		return table_found;
	}

	//set up kinect transform
	bool tferr = true;
	while (tferr) {
	    tferr = false;
		try {
	        //look for a transform from kinect to the torso
	        tf_listener_.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), transform_);
	    } catch (tf::TransformException &exception) {
	        ROS_ERROR("%s", exception.what());
	        tferr = true;
	        ros::Duration(0.5).sleep();
	        ros::spinOnce();
	    }
	}
	eigen_tf_ = cwru_pcl_utils_.transformTFToEigen(transform_);
	ROS_INFO("Found transform for kinect.");

	cwru_pcl_utils_.transform_selected_points_cloud(eigen_tf_);

	//use a calculated plane (normal vec and signed distance from origin) to find center of points
	Eigen::Vector3f table_normal_vector;
	double table_center_distance;
	cwru_pcl_utils_.fit_xformed_selected_pts_to_plane(table_normal_vector, table_center_distance);

	//multiply signed magnitude along normal vector to get to plane's center
	table_norm_ = table_normal_vector;
	table_center_ = table_normal_vector * table_center_distance;

	//other necessary values for arm goal
	table_major_axis_ = cwru_pcl_utils_.get_major_axis();
	table_centroid_ = cwru_pcl_utils_.get_centroid();

	return table_found;
}

//public method
//prepares the trajectories for moving arm to table and performing cleaning motion
//uses table_center_ with table_norm_ as goal
void RobotButler::prepare_motion() {
	//create an Eigen Affine to describe the table's center and normal vector
	for(int i = 0; i < 3; i++) {
		//origin of pose based on centroid
		origin_[i] = table_centroid_[i];
		//z - height axis aligned with table's norm (reversed)
		zdir_[i] = -table_norm_[i];
		//x axis chosen to be along computed major axis
		xdir_[i] = table_major_axis_[i];
	}
	//find y direction by crossing z-axis X x-axis
	ydir_ = zdir_.cross(xdir_);
	//load in goal matrix
	goal_matrix_.col(0) = xdir_;
	goal_matrix_.col(1) = ydir_;
	goal_matrix_.col(2) = zdir_;

	//transpose to Affine
	affine_target_.linear() = goal_matrix_;
	affine_target_.translation() = origin_;

	pose_target_.pose = transform_Affine_to_Pose_(affine_target_);
	ROS_INFO("Target goal pose computed for table.");

	// ROS_INFO("Moving to pre-pose to prepare for cleaning.");
// 	move_arm_to_pre_pose_();
// 	execute_arm_move_();
}

//public method
//passes calculated trajectories to bax_streamer_
//executes movement
void RobotButler::clean_table() {
	ROS_INFO("Moving arm to center of table for cleaning.");
	move_arm_to_goal_pose_(pose_target_);
	execute_arm_move_();

	//table displacement cleaning vectors
	Eigen::Vector3f left_move;
	left_move << -0.5, 0.0, 0.0;
	Eigen::Vector3f right_move;
	right_move << 2.0, 0.0, 0.0;
	Eigen::Vector3f up_move;
	up_move << 0.0, -1.0, 0.0;
	Eigen::Vector3f down_move;
	down_move << 0.0, 2.0, 0.0;

	ros::Duration(0.5).sleep();
	ROS_INFO("Cleaning.");

	ROS_INFO("Left to right first.");
	displace_arm_by_xyz_(left_move);
	execute_arm_move_();
	// displace_arm_by_xyz_(right_move);
	// execute_arm_move_();
	// displace_arm_by_xyz_(left_move);
	// execute_arm_move_();

	// ROS_INFO("Now up and down.");
	// displace_arm_by_xyz_(up_move);
	// execute_arm_move_();
	// displace_arm_by_xyz_(down_move);
	// execute_arm_move_();
	// displace_arm_by_xyz_(up_move);
	// execute_arm_move_();

	// ROS_INFO("All Done!");
	// move_arm_to_pre_pose_();
	// execute_arm_move_();
}

//private function
//transforms an eigen affine to a pose
//credit:: ArmMotionCommander class
geometry_msgs::Pose RobotButler::transform_Affine_to_Pose_(Eigen::Affine3d affine) {
	Eigen::Vector3d Oe;
    Eigen::Matrix3d Re;
    geometry_msgs::Pose pose;
    Oe = affine.translation();
    Re = affine.linear();

    Eigen::Quaterniond q(Re);
    pose.position.x = Oe(0);
    pose.position.y = Oe(1);
    pose.position.z = Oe(2);

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
}

//private function
//plans move to pre-pose
//credit:: ArmMotionCommander class
void RobotButler::move_arm_to_pre_pose_() {
    ROS_INFO("requesting a joint-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_JSPACE_PATH_CURRENT_TO_PRE_POSE;
    arm_ac_.sendGoal(cart_goal_);
    bool finished_before_timeout_ = arm_ac_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");       
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time;
    ROS_INFO("computed move time: %f",computed_arrival_time_);
}

//private function
//plans move to a specified pose
//credit:: ArmMotionCommander class
void RobotButler::move_arm_to_goal_pose_(geometry_msgs::PoseStamped pose) {
	ROS_INFO("requesting a cartesian-space motion plan");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_POSE;
    cart_goal_.des_pose_gripper_right = pose;
    arm_ac_.sendGoal(cart_goal_); // we could also name additional callback functions here, if desired
    bool finished_before_timeout_ = arm_ac_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
}

//private function
//displaces arm by an xyz vector, used to clean table
//credit:: ArmMotionCommander class
void RobotButler::displace_arm_by_xyz_(Eigen::Vector3f move) {
	ROS_INFO("requesting a cartesian-space motion plan along vector");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_PLAN_PATH_CURRENT_TO_GOAL_DP_XYZ;
    //must fill in desired vector displacement
    cart_goal_.arm_dp_right.resize(3);
    for (int i=0;i<3;i++) cart_goal_.arm_dp_right[i] = move[i];
    arm_ac_.sendGoal(cart_goal_); // we could also name additional callback functions here, if desired
    bool finished_before_timeout_ = arm_ac_.waitForResult(ros::Duration(2.0));
    ROS_INFO("return code: %d",cart_result_.return_code);
    if (!finished_before_timeout_) {
            ROS_WARN("giving up waiting on result");
        } 
    
    ROS_INFO("finished before timeout");
    if (cart_result_.return_code==cwru_action::cwru_baxter_cart_moveResult::RT_ARM_PATH_NOT_VALID) {
        ROS_WARN("right arm plan not valid");
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("unknown return code... not SUCCESS");
    }   
 
    //here if success return code
    ROS_INFO("returned SUCCESS from planning request");
    computed_arrival_time_= cart_result_.computed_arrival_time; //action_client.get_computed_arrival_time();
    ROS_INFO("computed move time: %f",computed_arrival_time_);
}

//private function
//executes the planned move for an arm
//credit:: ArmMotionCommander class
void RobotButler::execute_arm_move_() {
	ROS_INFO("requesting execution of planned path");
    cart_goal_.command_code = cwru_action::cwru_baxter_cart_moveGoal::RT_ARM_EXECUTE_PLANNED_PATH;
    arm_ac_.sendGoal(cart_goal_); // we could also name additional callback functions here, if desired
    bool finished_before_timeout_ = arm_ac_.waitForResult(ros::Duration(computed_arrival_time_+2.0));
    if (!finished_before_timeout_) {
        ROS_WARN("did not complete move in expected time");
    }
    if (cart_result_.return_code!=cwru_action::cwru_baxter_cart_moveResult::SUCCESS) {
        ROS_WARN("move did not return success; code = %d",cart_result_.return_code);
    }
    ros::Duration(computed_arrival_time_ + 5.0).sleep();
    //wait to execute loop
 //    double timer = 0.0;
 //    while(timer < computed_arrival_time_) {
 //    	ros::Duration(1.0).sleep();
 //    	timer += 1.0;
	// }
    ROS_INFO("move returned success");
}

//main method
int main(int argc, char** argv) {
	ros::init(argc, argv, "robot_butler");

	ros::NodeHandle nh;

	RobotButler rob(nh);

	if(ros::ok()) {
		//boolean loop to poll for selected data until succesful
		bool ready = rob.find_table();
		while(!ready) {
			ready = rob.find_table();
			ros::Duration(1.0).sleep();
			ros::spinOnce();
		}
		//once table is found, prepare motion
		rob.prepare_motion();
		//clean the table
		rob.clean_table();
	}

	return 0;
}