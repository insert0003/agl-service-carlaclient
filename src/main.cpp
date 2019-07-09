/*
 * Copyright (c) 2017 TOYOTA MOTOR CORPORATION
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

#include <mutex>
#include <json.h>
#include <pthread.h>

extern "C" {
#include <afb/afb-binding.h>
#include <systemd/sd-event.h>
}

#include "carlaclient.hpp"
// #include "debugmsg.hpp"

carla::CarlaClient *g_carlaclient;
std::mutex binding_m;

void *dataReceiveThread(void *ptr)
{
	carla::CarlaClient *carlaClient = (carla::CarlaClient *)ptr;
	// carlaClient->connect_server("192.168.160.168", 12345);
	carlaClient->connect_server();
	return nullptr;
}

static int init(afb_api_t api)
noexcept
{
	g_carlaclient = new carla::CarlaClient();
	if(g_carlaclient == nullptr)
	{
	    return 0;
	}
	else
	{
		g_carlaclient->init();

		pthread_t id;
	    if (pthread_create(&id, NULL, dataReceiveThread, g_carlaclient)) {
	        return -1;
	    }
	    return 0;
	}
}

void carlaclient_subscribe(afb_req_t req)
noexcept
{
	std::lock_guard<std::mutex> guard(binding_m);
	if(g_carlaclient == nullptr)
	{
		afb_req_fail(req, "failed", "Binding not initialized, did the compositor die?");
		return;
	}

	try
	{
		json_object *jreq = afb_req_json(req);
		json_object *j = nullptr;
		if(!json_object_object_get_ex(jreq, "event", &j))
		{
			afb_req_fail(req, "failed", "Need char const* argument event");
			return;
		}
		carla::CarlaClient::EventType event_id = (carla::CarlaClient::EventType)json_object_get_int(j);
		bool ret = g_carlaclient->subscribe(req, event_id);

		if(!ret)
		{
			afb_req_fail(req, "failed", "Error: afb_req_subscribe()");
			return;
		}
		afb_req_success(req, NULL, "success");
	}
	catch(std::exception &e)
	{
		afb_req_fail_f(req, "failed", "Uncaught exception while calling wm_subscribe: %s", e.what());
		return;
	}
}

void carlaclient_demo(afb_req_t req)
noexcept
{
	fprintf(stderr, "carla-service-carlaclient_demo\n");
	bool ret = true;

	std::lock_guard<std::mutex> guard(binding_m);
	if(g_carlaclient == nullptr)
	{
		afb_req_fail(req, "failed", "Binding not initialized, did the compositor die?");
		return;
	}

	try
	{
		json_object *jreq = afb_req_json(req);
		json_object *j = nullptr;
		if(!json_object_object_get_ex(jreq, "demo", &j))
		{
			afb_req_fail(req, "failed", "Need char const* argument event");
			return;
		}
		const char *demo = json_object_get_string(j);
		ret = g_carlaclient->set_demo_status(demo);

		if(!ret)
		{
			afb_req_fail(req, "failed", "Error: afb_req_subscribe()");
			return;
		}
		afb_req_success(req, NULL, "success");
	}
	catch(std::exception &e)
	{
		afb_req_fail_f(req, "failed", "Uncaught exception while calling wm_subscribe: %s", e.what());
		return;
	}
}

void carlaclient_set_amazon_code(afb_req_t req)
noexcept
{
	fprintf(stderr, "carla-service-carlaclient_set_amazon_code\n");
	bool ret = true;

	std::lock_guard<std::mutex> guard(binding_m);
	if(g_carlaclient == nullptr)
	{
		afb_req_fail(req, "failed", "Binding not initialized, did the compositor die?");
		return;
	}

	try
	{
		json_object *jreq = afb_req_json(req);
		json_object *j = nullptr;
		if(!json_object_object_get_ex(jreq, "amazon_code", &j))
		{
			afb_req_fail(req, "failed", "Need char const* argument event");
			return;
		}
		const char *amazon_code = json_object_get_string(j);
		ret = g_carlaclient->set_amazon_code(amazon_code);

		if(!ret)
		{
			afb_req_fail(req, "failed", "Error: afb_req_subscribe()");
			return;
		}
		afb_req_success(req, NULL, "success");
	}
	catch(std::exception &e)
	{
		afb_req_fail_f(req, "failed", "Uncaught exception while calling wm_subscribe: %s", e.what());
		return;
	}
}

const afb_verb_t carlaclient_verbs[]
= {
	{	.verb = "subscribe", .callback = carlaclient_subscribe},
	{	.verb = "demo", .callback = carlaclient_demo},
	{	.verb = "set_amazon_code", .callback = carlaclient_set_amazon_code},
	{}};

extern "C" const afb_binding_t afbBindingExport
= {
	.api = "carlaclient",
	.specification = "carlaclient",
	.info = "carlaclient",
	.verbs = carlaclient_verbs,
	.preinit = nullptr,
	.init = init,
	.onevent = nullptr,
	.userdata = nullptr,
	.provide_class = nullptr,
	.require_class = nullptr,
	.require_api = nullptr,
	.noconcurrency = 0
};
