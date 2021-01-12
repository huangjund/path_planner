#include "ccserver.h"

#include <fstream>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <condition_variable>
#include <geometry_msgs/PoseArray.h>

/*====================================PlannerAgvManager========================================*/

void PlannerAgvManager::checkAndAssign() {
	auto agvs = getAvailableAgvs();
	if (agvs.empty())
	{
		return;
	}
	else
	{
		cout << agvs[0]->getAgvId() << endl;
		if (!tasks_.empty()) {
			for (auto iter = tasks_.cbegin(); iter != tasks_.cend(); iter++)
			{
				agvs[0]->assignTask(*iter);
			}
			tasks_.clear();
		}
	}
}

void PlannerAgvManager::onTaskCompletedOrCancelled(
	const std::string agv_id, 
	int64 task_id, 
	const std::map<std::string, std::string>& info){
	cout << "agv_id:" << agv_id << " task_id:" << task_id<<" task info:";

	for (auto iter = info.begin(); iter != info.end(); iter++)
	{
		cout << iter->first + ":" + iter->second;
	}

	checkAndAssign();
}
/*=======================Planner Control Center===========================================*/

PlannerCC::PlannerCC(int port, std::string sensor_ip, int sensor_port)
	: sensor_started_(false), start_loading_(false),
	httpClient_(sensor_ip, sensor_port),
	client_(httpClient_, jsonrpccxx::version::v2),goalReady_(false),
	refresh_thread_(500), agv_manager_(port), ready_(false), main_thread_(0)
{
	refresh_thread_.setCyclicFunc(std::bind(&PlannerCC::refreshSensorDetection, this));
	refresh_thread_.setThreadExit(std::bind(&PlannerCC::closeSensorDetection, this));
	main_thread_.setCyclicFunc(std::bind(&PlannerCC::main, this));

	sub_path_ = node_.subscribe("/PlannerToinServer/Path", 1, &PlannerCC::generateTaskLists, this);
	pos_server_ = node_.advertiseService("inServerToPlanner/Pos", &PlannerCC::getAgvPos,this);
	pub_Obs_ = node_.advertise<geometry_msgs::PoseArray>("/inServerToPlanner/Obstacles",1);

	agv_manager_.start();
	refresh_thread_.run();
	main_thread_.run();
}

void PlannerCC::main()
{
	cv::Mat img;
	if (!agv_manager_.getTaskGroups().empty())
		img = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 255, 0));
	else
		img = cv::Mat(640, 640, CV_8UC3, cv::Scalar(0, 0, 255));
	imshow("press b to detect sensor", img);
	int key = waitKey(100);
	if (key == 'b')
	{
		start_loading_ = true;
	}
	agv_manager_.checkAndAssign();
}

void PlannerCC::generateTaskLists(const std_msgs::String::ConstPtr& msg)
{
	auto msgStr = msg->data;
	std::string temp;
	std::list<AgvTask> tasks;
	vn::communication::AgvTask task;
	// split the whole string into several tasks
	for(auto i = msgStr.cbegin(); i != msgStr.cend() - 1; ++i) {
		if ((*i) == '\n') {
			task.task_type = TaskType::kTaskTypeMoveTo;
			task.task_id = task_id_++;
			task.task_info.emplace("PathString", temp);
			tasks.push_back(std::move(task));
			temp.clear();
		}
		else {
			temp += (*i);
		}
	}
	task.task_type = TaskType::kTaskTypeMoveTo;
	task.task_id = task_id_++;
	task.task_info.emplace("PathString", temp);
	tasks.push_back(std::move(task));

	agv_manager_.addTaskGroup(tasks);
}

bool PlannerCC::getAgvPos(gazebo_msgs::GetModelState::Request& req,
													gazebo_msgs::GetModelState::Response& res) {
	if (req.model_name=="agvpose") {
		auto agvs = agv_manager_.getAvailableAgvs();
		if (agvs.size()) {
			auto pose = agvs[0]->getPose();
			res.pose.position.x = pose.x();
			res.pose.position.y = pose.y();
			res.pose.position.z = 0;
			auto quat = tf::createQuaternionFromYaw(pose.angle());
			res.pose.orientation.x = quat.x();
			res.pose.orientation.y = quat.y();
			res.pose.orientation.z = quat.z();
			res.pose.orientation.w = quat.w();
		}
		else {
			cout << "no available AGV" << endl;
		}
	}
	else if (req.model_name=="goalpose") {
		start_loading_ = true;
		std::unique_lock<std::mutex> locker(mutexGoalPos_);
		condition_.wait(locker, [this]{return goalReady_;});
		goalReady_ = false;
		res.pose.position.x = goalPos_.x;
		res.pose.position.y = goalPos_.y;
		res.pose.position.z = goalPos_.theta;
		locker.unlock();
	}
	return true;
}

void PlannerCC::tryStartMonitor()
{
	if (!sensor_started_)
	{
		if (!client_.CallMethod<bool>(++rpc_id_, "startMonitor"))
		{
			refresh_thread_.asychStop();
			cerr << "fail to start sensor" << endl;
		}
		sensor_started_ = true;
	}
}

// 
void PlannerCC::refreshSensorDetection()
{
	try
	{
		auto agvs = agv_manager_.getAvailableAgvs();
		// if is demo 2
		if (start_obs_ && agvs.size()) {
			// get agv pose, no checker here
			auto pose = agvs[0]->getPose();
			double t = pose.angle();
			double rotation[2][2] = {std::cos(t), -std::sin(t),
															std::sin(t),  std::cos(t)};

			// call jsonrpc to get obstaclePose
			std::vector<ObstaclePose> obs = client_.CallMethod<
													std::vector<ObstaclePose>>(++rpc_id_, "GetObstaclePose");
			geometry_msgs::PoseArray obs_array;
			obs_array.header.frame_id = "map";
			obs_array.header.seq = 1;
			geometry_msgs::Pose temp;
			for(auto i = obs.cbegin(); i != obs.cend(); ++i) {
				double x = i->hrzDis, y = i->vtcDis;
				temp.position.x = pose.x() + x*rotation[0][0] + y*rotation[0][1];
				temp.position.y = pose.y() + x*rotation[1][0] + y*rotation[1][1];
				std::cout << "x: " << temp.position.x << "y: " << temp.position.y << std::endl;
				obs_array.poses.push_back(std::move(temp));
			}
			pub_Obs_.publish(obs_array);
		}

		// if start loading informtion from sensor
		// then take the goal
		if (start_loading_)
		{
			if (!agv_manager_.getTaskGroups().empty())
			{
				cout << "[PlannerCC]:tasks are ongoing" << endl;
				start_loading_ = false;
				return;
			}

			std::lock_guard<std::mutex> locker(mutexGoalPos_);
			// get result
			goalPos_ = client_.CallMethod<StorageBin>(++rpc_id_, "getStorageBin");
			cout << "x: " << goalPos_.x << "y: " << goalPos_.y << "z: " << goalPos_.z << endl;
			start_loading_ = false;
			goalReady_ = true;
		}
		condition_.notify_one();

		closeSensorDetection();
	}
	catch (jsonrpccxx::JsonRpcException e)
	{
		start_loading_ = false;
		if (e.Code() == -32003)
		{
			cout << "lost connection to sensor" << endl;
			sensor_started_ = false;
		}
		else
		{
			cout << e.Message() << endl;
			refresh_thread_.asychStop();
		}
	}

}

void PlannerCC::closeSensorDetection()
{
	sensor_started_ = false;
}

/*==========================Control Center Configuration=====================================*/

bool CCConfig::loadConfigImpl(tinyxml2::XMLElement* element) {
	return true;
}

int main(int argc, char* argv[]) {
	CCConfig config;

	ros::init(argc,argv,"innerServer");
	PlannerCC cc(config.getServerPort(), config.getTruckDetectionIp(), config.getTruckDetectionPort());
	ros::spin();

	return 0;
}