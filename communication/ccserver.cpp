#include "ccserver.h"

#include <fstream>

struct StorageBin
{
	int id;
	float x;
	float y;
	float z;
	float theta;
	bool flip;
};
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
		// std::list<AgvTask> next_task_group;
		// {
		// 	lock_guard<mutex> locker(tasks_mutex_);
		// 	if (!tasks_.empty())
		// 	{
		// 		next_task_group = tasks_;
		// 	}
		// 	else
		// 	{
		// 		return;
		// 	}
		// }

		// for (auto iter = next_task_group.begin(); iter != next_task_group.end(); iter++)
		// {
		// 	agvs[0]->assignTask(*iter);
		// }
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
	client_(httpClient_, jsonrpccxx::version::v2),
	refresh_thread_(500), agv_manager_(port), ready_(false), main_thread_(0)
{
	refresh_thread_.setCyclicFunc(std::bind(&PlannerCC::refreshSensorDetection, this));
	refresh_thread_.setThreadExit(std::bind(&PlannerCC::closeSensorDetection, this));
	main_thread_.setCyclicFunc(std::bind(&PlannerCC::main, this));
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

void PlannerCC::generateTaskLists()
{
	stringstream ss;
	TaskListParser parser;
	std::list<vn::communication::AgvTask> tasks;
	try
	{
		tasks = parser.parseFromFile(ss.str(), task_id_);
	}
	catch (nlohmann::detail::exception& e)
	{
		cout << "[PlannerCC]:fail to load task list " << endl;
	}
	agv_manager_.addTaskGroup(tasks);
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

			tryStartMonitor();
			// get result
			// auto parks = client_.CallMethod<std::vector<StorageBin>>(++rpc_id_, "getResult");

			// if (parks.empty())
			// {
			// 	INFO(logger) << "can not detect truck" << endl;
			// 	return;
			// }

			// for (auto park : parks)
			// {
			// 	park.theta = park.theta / 180.0 * M_PI;
			// 	INFO(logger) << "id:" << park.id << " x" << park.x << " y" << park.y << " z" << park.z << " theta" << park.theta << endl;
			// }

			start_loading_ = false;

			// generateTaskLists(parks);
		}

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
	if (sensor_started_)
	{
		if (!client_.CallMethod<bool>(++rpc_id_, "stopMonitor"))
		{
			cout << "[PlannerCC]:fail to start sensor" << endl;
		}
	}
	sensor_started_ = false;
}

/*==========================Control Center Configuration=====================================*/

bool CCConfig::loadConfigImpl(tinyxml2::XMLElement* element) {
	return true;
}

int main(int argc, char* argv[]) {
	CCConfig config;

	PlannerCC cc(config.getServerPort(), config.getTruckDetectionIp(), config.getTruckDetectionPort());

	pause();
}