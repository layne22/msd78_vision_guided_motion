//Assignment 8 where a point cloud is used to wipe a table
//baxter_traj_streamer will handle motion for the baxter sim
//cwru_pcl_utils will provide point cloud processing to find motion targets
//written by Matt Dobrowsky

#include <ros/ros.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>
#include <cwru_action/trajAction.h>

class RobotButler {
private:
	ros::NodeHandle nh_;

	CwruPclUtils cwru_pcl_utils_;

	Baxter_traj_streamer bax_streamer_;

	Eigen::Vector3f table_center_;

	//trajectories for carrying out duty
	cwru_action::trajGoal traj_to_table_;
	cwru_action::trajGoal traj_to_clean_;

public:
	RobotButler(ros::NodeHandle nh);

	~RobotButler(void) { }

	//find for selected points to interpret table location
	//returns true if points were selected
	bool find_table();

	//loads trajectories for arm movement to:
	//		place arm over table
	//		cleaning motion on table
	void prepare_motion();

	//executes trajectories for prep and cleaning
	void clean_table();	
};

//constructor
RobotButler::RobotButler(ros::NodeHandle nh) : nh_(nh), cwru_pcl_utils_(&nh), bax_streamer_(&nh) {

}

//public method
//returns true when a group of pcl points have been selected as the "table"
//returns false when no points selected
bool RobotButler::find_table() {
	bool table_found = false;


	return table_found;
}

//public method
//prepares the trajectories for moving arm to table and performing cleaning motion
//uses table_center_ as goal
void RobotButler::prepare_motion() {

}

//public method
//passes calculated trajectories to bax_streamer_
//executes movement
void RobotButler::clean_table() {

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
			rob.find_table();
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