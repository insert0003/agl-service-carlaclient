/*
 * Copyright (c) 2019 TOYOTA MOTOR CORPORATION
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef TMCAGL_CARLA_CLIENT_HPP
#define TMCAGL_CARLA_CLIENT_HPP

#include <map>
#include <string.h>
#include <mutex>
#include <vector>
#include <pthread.h>

extern "C" {
#include <afb/afb-binding.h>
}

#include "cansender.hpp"

namespace carla
{	

class CarlaClient
{
public:
	enum EventType
	{
		Event_Val_Min = 0,
		Event_PositionUpdated = Event_Val_Min,
		Event_Val_Max,
	};
	
	explicit CarlaClient();
	~CarlaClient();

	int init();
	int connect_server();
	bool subscribe(afb_req_t req, EventType event_id);
	bool set_demo_status(const char *status);

private:
	CarlaClient(CarlaClient const&) = delete;
	CarlaClient& operator=(CarlaClient const&) = delete;
	CarlaClient(CarlaClient&&) = delete;
	CarlaClient& operator=(CarlaClient&&) = delete;

	int loadServer();
	int inputJsonFilie(const char *file, json_object **obj);
	void emitPosition(const char* yaw, const char* longitude, const char* latitude);

private:
	std::map<std::string, afb_event_t> map_afb_event;

	std::string server_ip;
	int server_port;
	int reconnect_interval;
	int reconnect_times;
	int socketfd;

	CanSender cansender;

	bool demo_status_change;
	std::string demo_status;
	// std::mutex demo_m;
	pthread_mutex_t mtx;

	FILE *gps_file;
	std::vector<std::string> gps_data;
};

} // namespace carla

#endif  // !TMCAGL_CARLA_CLIENT_HPP
