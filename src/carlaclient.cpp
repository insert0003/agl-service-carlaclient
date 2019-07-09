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

#include <unistd.h>
#include <algorithm>
#include <json.h>
#include <vector>
#include <stdio.h>
#include <sys/socket.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sstream>

#include "carlaclient.hpp"
#include "debugmsg.hpp"

namespace carla
{

#define MAXLENGTH 1024
#define CARLA_SERVER_CONFIG "/etc/carla-server.json"

static const char kKeySpeed[] = "speed";
static const char kKeyEngineSpd[] = "engine_spd";
static const char kKeyGps[] = "gps";
static const char kKeyYaw[] = "yaw";
static const char kKeyLongitude[] = "longitude";
static const char kKeyLatitude[] = "latitude";

static const std::vector<std::string> kListEventName
{
	"positionUpdated",
};

CarlaClient::CarlaClient() :
socketfd(0),
demo_status_change(false),
demo_status(""),
amazon_code_change(false),
amazon_code(""),
demo_m()
{
	cansender.init();
}

CarlaClient::~CarlaClient()
{
	close( socketfd);
}

int CarlaClient::init()
{
	int ret = 0;

	for(int i = Event_Val_Min; i < Event_Val_Max; i++)
	{
		map_afb_event[kListEventName[i]] = afb_api_make_event(afbBindingV3root,
				kListEventName[i].c_str());
	}

	loadServer();

	return ret;
}

int CarlaClient::connect_server()
{
	struct sockaddr_in sockaddr;
	char readline[MAXLENGTH];
	char writeline[MAXLENGTH];
	int length;
	int reconnect_times_startup = reconnect_times;

	DBG_DEBUG(LOG_PREFIX, "Server: %s port : %d", server_ip.c_str(), server_port);

	memset(&sockaddr, 0, sizeof(sockaddr));
	sockaddr.sin_family = AF_INET;
	sockaddr.sin_port = htons(server_port);

	inet_pton(AF_INET, server_ip.c_str(), &sockaddr.sin_addr);

	while(true)
	{
		socketfd = socket(AF_INET, SOCK_STREAM, 0);
		int ret = connect(socketfd, (struct sockaddr*) &sockaddr, sizeof(sockaddr));
		if(ret < 0)
		{
			DBG_DEBUG(LOG_PREFIX, "Cannot connect the server err: %s errno : %d", strerror(errno), errno);
			DBG_DEBUG(LOG_PREFIX, "failed to reconnect, sleep %d sec with %d time(s) left", reconnect_interval, reconnect_times_startup);
			close(socketfd);
			sleep(reconnect_interval);
		}
		else
		{
			DBG_DEBUG(LOG_PREFIX, "succeeded to reconnect");
			break;
		}

		if(reconnect_times_startup == -1)
		{
			/* Nothing to do*/
		}
		else
		{
			reconnect_times_startup--;
			if(reconnect_times_startup == 0)
			{
				DBG_DEBUG(LOG_PREFIX, "Exceed max reconnect times");
				return -1;
			}
		}
	}

	while(true)
	{
		{
			std::lock_guard<std::mutex> guard(demo_m);
			if(demo_status_change)
			{
				DBG_DEBUG(LOG_PREFIX, "send demo_status: %s", demo_status.c_str());
				memset(writeline, 0, MAXLENGTH);
				sprintf(writeline, "{\"cmd\":\"demo\", \"val\":\"%s\"}", demo_status.c_str());
				DBG_DEBUG(LOG_PREFIX, "send string: %s", writeline);
				send(socketfd, writeline, strlen(writeline), 0);
				demo_status_change = false;
			}
			else if(amazon_code_change)
			{
				DBG_DEBUG(LOG_PREFIX, "send amazon_code: %s", amazon_code.c_str());
				memset(writeline, 0, MAXLENGTH);
				sprintf(writeline, "{\"cmd\":\"amazon_code\", \"val\":\"%s\"}", amazon_code.c_str());
				DBG_DEBUG(LOG_PREFIX, "send string: %s", writeline);
				send(socketfd, writeline, strlen(writeline), 0);
				amazon_code_change = false;
			}
		}

		memset(readline, 0, MAXLENGTH);
		length = recv(socketfd, readline, MAXLENGTH, 0);

		///
		if(length <= 0)
		{
			int reconnect_times_offline = reconnect_times;
			DBG_DEBUG(LOG_PREFIX, "recv error: return value: %d", length);
			DBG_DEBUG(LOG_PREFIX, "recv error: errno: %s", strerror(errno));

			close(socketfd);
			socketfd = socket(AF_INET, SOCK_STREAM, 0);

			while(true)
			{
				int ret = connect(socketfd, (struct sockaddr*) &sockaddr, sizeof(sockaddr));
				if(ret < 0)
				{
					DBG_DEBUG(LOG_PREFIX, "failed to reconnect, sleep %d sec with %d time(s) left", reconnect_interval, reconnect_times_offline);
					sleep(reconnect_interval);
				}
				else
				{
					DBG_DEBUG(LOG_PREFIX, "succeeded to reconnect");
					break;
				}

				if(reconnect_times_offline == -1)
				{
					/* Nothing to do*/
				}
				else
				{
					reconnect_times_offline--;
					if(reconnect_times_offline == 0)
					{
						DBG_DEBUG(LOG_PREFIX, "Exceed max reconnect times");
						return -1;
					}
				}
				
			}
		}
		///

		if(length >= MAXLENGTH)
		{
			length = MAXLENGTH - 1;
		}
		readline[length] = '\0';

		// fprintf(stderr, ">>>>> recv msg 1 from length:%d, client:%s\n", length,
		// 		readline);

		if(strlen(readline) <= 0)
		{
			DBG_DEBUG(LOG_PREFIX, "recv again");
			continue;
		}

		json_object* jobj = json_tokener_parse(readline);

		//		{"gps": {"latitude": "49.002756551435", "longitude": "8.001536315145"}}
		//DBG_INFO(LOG_PREFIX, "Recv msg length:%d, content:%s", length, json_object_get_string(jobj));

		if(jobj != nullptr)
		{
			json_object_object_foreach(jobj, key, val)
			{
				if(strcmp(key, kKeyGps) == 0)
				{
					json_object *jyaw;
					json_object *jlon;
					json_object *jlat;
					if(val)
					{
						json_object_object_get_ex(val, kKeyYaw, &jyaw);
						json_object_object_get_ex(val, kKeyLongitude, &jlon);
						json_object_object_get_ex(val, kKeyLatitude, &jlat);

						// fprintf(stderr, ">>>>> recv msg 3 jgps:%s, jlon:%s, jlat:%s\n",json_object_get_string(jgps),
						// 		json_object_get_string(jlon), json_object_get_string(jlat));
						if(jyaw && jlon && jlat)
						{
							// DBG_DEBUG(LOG_PREFIX, "GPS: %s %s", json_object_get_string(jlon), json_object_get_string(jlat));
							emitPosition(json_object_get_string(jyaw), json_object_get_string(jlon), json_object_get_string(jlat));
						}
					}
				}
				else if(strcmp(key, kKeySpeed) == 0)
				{
					int speed;
					if(val)
					{
						speed = json_object_get_int(val);
						// DBG_INFO(LOG_PREFIX, "Speed:%d", speed);
						cansender.updateValue(VEHICLE_SPEED, speed);
					}
				}
				else if(strcmp(key, kKeyEngineSpd) == 0)
				{
					int engine_speed;
					if(val)
					{
						engine_speed = json_object_get_int(val);
						// DBG_INFO(LOG_PREFIX, "Engine Speed:%d", engine_speed);
						cansender.updateValue(ENGINE_SPEED, engine_speed);
					}
				}
				else
				{
					DBG_ERROR(LOG_PREFIX, "Invalid msg!");
				}
			}
		}
		else
		{
			DBG_DEBUG(LOG_PREFIX, "Incomplete json data %s", readline);
		}
	}

	return 0;
}

bool CarlaClient::subscribe(afb_req_t req, EventType event_id)
{
	int ret = 0;

	if(event_id < Event_Val_Min || event_id >= Event_Val_Max)
	{
		return false;
	}

	ret = afb_req_subscribe(req, map_afb_event[kListEventName[event_id]]);

	if(ret)
	{
		return false;
	}

	return true;
}

bool CarlaClient::set_demo_status(const char *status)
{
	DBG_DEBUG(LOG_PREFIX, "demo_status 1: %s", status);
	std::lock_guard<std::mutex> guard(demo_m);
	demo_status_change = true;
	if(strcmp(status, "true") == 0)
	{
		demo_status = "true";
	}
	else if(strcmp(status, "false") == 0)
	{
		demo_status = "false";
	}
	DBG_DEBUG(LOG_PREFIX, "demo_status 2: %s", demo_status.c_str());

	return true;
}

bool CarlaClient::set_amazon_code(const char *code)
{
	DBG_DEBUG(LOG_PREFIX, "set_amazon_code: %s", code);
	std::lock_guard<std::mutex> guard(demo_m);
	amazon_code_change = true;
	amazon_code = code;

	return true;
}

int CarlaClient::loadServer()
{
	std::string file_name(CARLA_SERVER_CONFIG);

    // Load server
    json_object *json_obj;
    int ret = this->inputJsonFilie(file_name.c_str(), &json_obj);
    if(0 > ret)
    {
        DBG_ERROR(LOG_PREFIX, "Could not open server config");
        return -1;
    }
    DBG_INFO(LOG_PREFIX, "json_obj dump:%s", json_object_get_string(json_obj));

	// Get ip
    json_object *json_ip;
    if(!json_object_object_get_ex(json_obj, "ip", &json_ip))
    {
        DBG_ERROR(LOG_PREFIX, "Parse Error!!");
        return -1;
    }
	else
	{
		server_ip = json_object_get_string(json_ip);
	}

	// Get port
	json_object *json_port;
    if(!json_object_object_get_ex(json_obj, "port", &json_port))
    {
        DBG_ERROR(LOG_PREFIX, "Parse Error!!");
        return -1;
    }
	else
	{
		server_port = json_object_get_int(json_port);
	}

	// Get interval
	json_object *json_interval;
    if(!json_object_object_get_ex(json_obj, "reconnect_interval", &json_interval))
    {
        DBG_ERROR(LOG_PREFIX, "Parse Error!!");
        return -1;
    }
	else
	{
		reconnect_interval = json_object_get_int(json_interval);
	}

	// Get times
	json_object *json_times;
    if(!json_object_object_get_ex(json_obj, "reconnect_times", &json_times))
    {
        DBG_ERROR(LOG_PREFIX, "Parse Error!!");
        return -1;
    }
	else
	{
		reconnect_times = json_object_get_int(json_times);
	}

    return 0;
}

int CarlaClient::inputJsonFilie(const char *file, json_object **obj)
{
    const int input_size = 128;
    int ret = -1;

    DBG_INFO(LOG_PREFIX, "Input file: %s", file);

    // Open json file
    FILE *fp = fopen(file, "rb");
    if(nullptr == fp)
    {
        DBG_ERROR(LOG_PREFIX, "Could not open file");
        return ret;
    }

    // Parse file data
    struct json_tokener *tokener = json_tokener_new();
    enum json_tokener_error json_error;
    char buffer[input_size];
    int block_cnt = 1;
    while(1)
    {
        size_t len = fread(buffer, sizeof(char), input_size, fp);
        *obj = json_tokener_parse_ex(tokener, buffer, len);
        if(nullptr != *obj)
        {
            DBG_INFO(LOG_PREFIX, "File input is success");
            ret = 0;
            break;
        }

        json_error = json_tokener_get_error(tokener);
        if((json_tokener_continue != json_error) || (input_size > len))
        {
            DBG_ERROR(LOG_PREFIX, "Failed to parse file (byte:%d err:%s)",
                      (input_size * block_cnt), json_tokener_error_desc(json_error));
            DBG_ERROR(LOG_PREFIX, "\n%s", buffer);
            *obj = nullptr;
            break;
        }
        block_cnt++;
    }

    // Close json file
    fclose(fp);

    // Free json_tokener
    json_tokener_free(tokener);

    return ret;
}

void CarlaClient::emitPosition(const char* yaw, const char* longitude, const char* latitude)
{
	json_object* j = nullptr;
	afb_event_t event = map_afb_event[kListEventName[Event_PositionUpdated]];

#if 0
	std::ostringstream lon;
	std::ostringstream lat;
	lon.precision(15);
	lat.precision(15);
	lon << longitude;
	lat << latitude;

	j = json_object_new_object();
	json_object_object_add(j, kKeyLongitude, json_object_new_string(
					lon.str().c_str()));
	json_object_object_add(j, kKeyLatitude, json_object_new_string(
					lat.str().c_str()));
#else
	j = json_object_new_object();
	json_object_object_add(j, kKeyYaw, json_object_new_string(yaw));
	json_object_object_add(j, kKeyLongitude, json_object_new_string(longitude));
	json_object_object_add(j, kKeyLatitude, json_object_new_string(latitude));
#endif
	if(afb_event_is_valid(event))
	{
		int ret = afb_event_push(event, j);
		if(ret != 0)
		{
			// fprintf(stderr, ">>>>> carla: afb_event_push failed\n");
		}
	}
	else
	{
		// fprintf(stderr, ">>>>> carla: failed to send");
	}
}

} // namespace carla
