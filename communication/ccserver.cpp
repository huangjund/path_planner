#include "ccserver.h"

#include <fstream>
#include <tf/transform_datatypes.h>
#include <condition_variable>


void to_json(nlohmann::json& j, const StorageBin& p) { 
	j = nlohmann::json{ 
		{"id", p.id}, 
		{"x", p.x}, 
		{"y", p.y}, 
		{"z", p.z},
		{"theta", p.theta},
		{"flip", p.flip}
	};
}
void from_json(const nlohmann::json& j, StorageBin& p) {
	j.at("id").get_to(p.id);
	j.at("x").get_to(p.x);
	j.at("y").get_to(p.y);
	j.at("z").get_to(p.z);
	j.at("theta").get_to(p.theta);
	j.at("flip").get_to(p.flip);
}

/*====================================PlannerAgvManager========================================*/

void PlannerAgvManager::checkAndAssign() {
	auto agvs = getAvailableAgvs();
	if (agvs.empty())
	{
		return;
	}
	else
	{
		if (!tasks_.empty()) {
			for (auto iter = tasks_.begin(); iter != tasks_.end(); iter++)
			{
				agvs[0]->assignTask(*iter);
			}
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
	stringstream ss;
	ss << msg;
	vn::communication::AgvTask task;
	task.task_type = TaskType::kTaskTypeMoveTo;
	task.task_info.emplace("PathString", ss.str());
	std::list<vn::communication::AgvTask> tasks;
	tasks.push_back(task);
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

// try get goal pose from Sensor
void PlannerCC::refreshSensorDetection()
{
	try
	{
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