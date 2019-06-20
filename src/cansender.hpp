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

#ifndef TMCAGL_CAN_SENDER_HPP
#define TMCAGL_CAN_SENDER_HPP

#include <sys/types.h>
#include <stdint.h>
#include <json-c/json.h>
#include <pthread.h>

#include "canencoder.hpp"

namespace carla
{

#define STEERING_WHEEL_JSON	"/etc/steering_wheel.json"
#define BUS_MAP_CONF "/etc/dev-mapping.conf"

#define VEHICLE_SPEED				"VehicleSpeed"
#define ENGINE_SPEED				"EngineSpeed"
#define ACCELERATOR_PEDAL_POSITION	"AcceleratorPedalPosition"
#define TRANSMISSION_GEAR_INFO		"TransmissionGearInfo"
#define TRANSMISSION_MODE			"TransmissionMode"
#define STEERING_WHEEL_ANGLE		"SteeringWheelAngle"
#define TURN_SIGNAL_STATUS			"TurnSignalStatus"
#define LIGHT_STATUS_BRAKE			"LightStatusBrake"

struct wheel_info_t
{
	unsigned int   nData;
	struct prop_info_t property[0];
	/* This is variable structure */
};

class CanSender
{
public:
    explicit CanSender();
	~CanSender();

    int init();
    void updateValue(const char *prop, int val);

private:
    int initConfig();
    int initTransmissionLoop();
    int readTransBus(void);
    int readJsonConfig();
    int wheel_define_init(const char *fname);
    int wheel_gear_para_init(const char *fname);
    int parse_json(json_object *obj);
    int parse_gear_para_json(json_object *obj);
    int parse_propertys(json_object *obj_propertys);
    int parse_property(int idx, json_object *obj_property);

private:
    struct wheel_info_t *wheel_info;
    pthread_t thread_id;
};

} // namespace carla

#endif  // !TMCAGL_CAN_SENDER_HPP
