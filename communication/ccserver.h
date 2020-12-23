#ifndef _HYBRID_A_STAR_CC_SERVER_H_
#define _HYBRID_A_STAR_CC_SERVER_H_

#include <unistd.h>

#include <VnUtils/core/logger.hh>
#include <VnUtils/core/cyclic_thread.hh>
#include <VnUtils/core/xml_configurable.hh>

#include <VnUtils/geometry/position_2d.hh>

#include <VnUtils/communication/agv.hh>
#include <VnUtils/communication/agv_manager.hh>
#include <VnUtils/communication/task_list_parser.hh>

#include <jsonrpccxx/cpphttplibconnector.hpp>
#include <jsonrpccxx/client.hpp>

#include <opencv2/highgui.hpp>

using namespace vn::communication;
using namespace vn::core;
using namespace vn::geometry;
using namespace jsonrpccxx;
using namespace std;
using namespace cv;
using namespace tinyxml2;

class CCConfig : public vn::core::XmlConfigurable
{
 public:
	CCConfig(): 
    port_(9102),
    truck_detection_ip_("localhost"),
    truck_detection_port_(8484){}

	int getServerPort()
	{
		return port_;
	}
	std::string getTruckDetectionIp()
	{
		return truck_detection_ip_;
	}
	int getTruckDetectionPort()
	{
		return truck_detection_port_;
	}
 protected:

	/**
	 * @brief       Implementation of loading of the configurations.
	 * @param[in]   element XML element to read from.
	 * @return success or not
	 */
	virtual bool loadConfigImpl(tinyxml2::XMLElement* element) override;

 private:
	int port_;
	std::string truck_detection_ip_;
	int truck_detection_port_;
};

class PlannerAgvManager : public AgvManager
{
 public:
	PlannerAgvManager(int port, int timeout = 10) : 
		AgvManager(port,timeout) {}

	void addTaskGroup(std::list<AgvTask> tasks){
		lock_guard<mutex> locker(tasks_mutex_);
		tasks_ = tasks;
	}

	void checkAndAssign();

	std::list<AgvTask> getTaskGroups()
	{
		return tasks_;
	}
 private:
	virtual void onTaskCompletedOrCancelled(
      const std::string agv_id, 
      int64 task_id, 
      const std::map<std::string, std::string>& info);

 protected:
	virtual void onTaskCompleted(const std::string agv_id, int64 task_id, const std::map<std::string, std::string>& info)
	{
		onTaskCompletedOrCancelled(agv_id, task_id, info);
	}

	virtual void onTaskCancelled(const std::string agv_id, int64 task_id, const std::map<std::string, std::string>& info)
	{
		onTaskCompletedOrCancelled(agv_id, task_id, info);
	}
 protected:
	std::mutex tasks_mutex_;
	std::list<AgvTask> tasks_;
};

class PlannerCC
{
 public:
	PlannerCC(int port, std::string sensor_ip, int sensor_port);

	~PlannerCC() {
		refresh_thread_.stop();
		main_thread_.stop();
	}

 private:
	void main();

	/*the entrence of path, generated by planner,
		to AGV manager*/
	void generateTaskLists();

	void tryStartMonitor();

	void refreshSensorDetection();

	void closeSensorDetection();

 private:
	PlannerAgvManager agv_manager_;

	static int64 task_id_;

	CyclicThread main_thread_;
	bool ready_;

	CyclicThread refresh_thread_;			/*!< thread that monitors the detection of the truck */
	bool sensor_started_;			/*!< whether the sensor module is started */
	bool start_loading_;					/*!< try start load the sensor, if the sensor is not found, will be set to false */
	static int rpc_id_;						/*!< rpc call id */
	CppHttpLibClientConnector httpClient_;	/*!< http client of sensor */
	JsonRpcClient client_;					/*!< json rpc client of sensor */
};
int PlannerCC::rpc_id_ = 0;	// rpc id counts for the callings to the sensor?
int64 PlannerCC::task_id_ = 1;

#endif