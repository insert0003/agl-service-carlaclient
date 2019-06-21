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

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <systemd/sd-event.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>

#include "cansender.hpp"
#include "debugmsg.hpp"

namespace carla
{

double gearRatio[8] =
{
	0.0,	//Neutral
	1.0/4.12,	//First
	1.0/2.84,	//Second
	1.0/2.28,	//Third
	1.0/1.45,	//Fourth
	1.0/1.0,	//Fifth
	1.0/0.69,	//Sixth
	1.0/3.21	//Reverse
};

struct transmission_bus_conf
{
	char *hs;
	char *ls;
};

static struct transmission_bus_conf trans_conf;

static void *transmission_event_loop(void *args)
{
	int s; /* can raw socket */
	unsigned int required_mtu;
	int mtu;
	int enable_canfd = 1;
	struct sockaddr_can addr;
	struct canfd_frame frame;
	struct ifreq ifr;
//	int retry = 0;

	/* open socket */
	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("open socket failed");
		return 0;
	}

	addr.can_family = AF_CAN;
	strcpy(ifr.ifr_name, trans_conf.hs);
	/* wait until hs device start */
	while(1) {
		if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
			carla::clear();		/* clear transmission msg queue */
			perror("SIOCGIFINDEX");
			sleep(2);
		}
		else
		{
			break;
		}
	}

	addr.can_ifindex = ifr.ifr_ifindex;

	/* disable default receive filter on this RAW socket */
	/* This is obsolete as we do not read from the socket at all, but for */
	/* this reason we can remove the receive list in the Kernel to save a */
	/* little (really a very little!) CPU usage.                          */
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("bind");
		return 0;
	}

	while(1)
	{
		struct can_data_t* p = carla::pop();
		if(p == NULL)
		{
			/* sleep 150ms */
			usleep(150000);
			continue;
		}

		/* parse CAN frame */
		required_mtu = carla::parse_canframe(p->dat, &frame);
		free(p);
		if (!required_mtu){
			fprintf(stderr, "\nWrong CAN-frame format! Try:\n\n");
			fprintf(stderr, "    <can_id>#{R|data}          for CAN 2.0 frames\n");
			fprintf(stderr, "    <can_id>##<flags>{data}    for CAN FD frames\n\n");
			fprintf(stderr, "<can_id> can have 3 (SFF) or 8 (EFF) hex chars\n");
			fprintf(stderr, "{data} has 0..8 (0..64 CAN FD) ASCII hex-values (optionally");
			fprintf(stderr, " separated by '.')\n");
			fprintf(stderr, "<flags> a single ASCII Hex value (0 .. F) which defines");
			fprintf(stderr, " canfd_frame.flags\n\n");
			fprintf(stderr, "e.g. 5A1#11.2233.44556677.88 / 123#DEADBEEF / 5AA# / ");
			fprintf(stderr, "123##1 / 213##311\n     1F334455#1122334455667788 / 123#R ");
			fprintf(stderr, "for remote transmission request.\n\n");
			continue;
		}

		if (required_mtu > CAN_MTU) {

			/* check if the frame fits into the CAN netdevice */
			if (ioctl(s, SIOCGIFMTU, &ifr) < 0) {
				perror("SIOCGIFMTU");
				continue;
			}
			mtu = ifr.ifr_mtu;

			if (mtu != CANFD_MTU) {
				fprintf(stderr, "CAN interface ist not CAN FD capable - sorry.\n");
				continue;
			}

			/* interface is ok - try to switch the socket into CAN FD mode */
			if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
				       &enable_canfd, sizeof(enable_canfd))){
				fprintf(stderr, "error when enabling CAN FD support\n");
				continue;
			}

			/* ensure discrete CAN FD length values 0..8, 12, 16, 20, 24, 32, 64 */
			frame.len = carla::can_dlc2len(carla::can_len2dlc(frame.len));
		}

		/* send frame */
		if (write(s, &frame, required_mtu) != required_mtu) {
			perror("write");
		}
	}
}

CanSender::CanSender()
{
}

CanSender::~CanSender()
{

}

int CanSender::init()
{
    if(initConfig())
    {
        DBG_ERROR(LOG_PREFIX, "init config failed");
		return -1;
    }

    ///
    for(uint i = 0; i < wheel_info->nData; i++)
    {
        DBG_INFO(LOG_PREFIX, "wheel_info %s %s %d", wheel_info->property[i].name, wheel_info->property[i].can_id, wheel_info->property[i].bit_pos);
    }
    ///
    carla::init_can_encoder();

    if(initTransmissionLoop())
    {
        DBG_ERROR(LOG_PREFIX, "init loop failed");
		return -1;
    }

    return 0;
}

int CanSender::initConfig()
{
    if(readTransBus())
	{
		DBG_ERROR(LOG_PREFIX, "read file  (/etc/dev-mapping.conf) failed");
		return -1;
	}

    if(readJsonConfig())
	{
		DBG_ERROR(LOG_PREFIX, "read config file failed");
		return -1;
	}

    return 0;
}

int CanSender::initTransmissionLoop(void)
{
    int ret = pthread_create(&thread_id, NULL, carla::transmission_event_loop, NULL);
	if(ret != 0)
    {
		DBG_ERROR(LOG_PREFIX,  "Cannot run eventloop due to error:%d", errno);
		return -1;
	}

	return 0;
}

int CanSender::readTransBus(void)
{
	char *line = NULL;
	char bus_name[4], bus_val[10];
	size_t len = 0;
	ssize_t read;

	FILE *fp = fopen(BUS_MAP_CONF,"r");
	if(fp == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot read /etc/dev-mapping.conf");
		return -1;
	}

	while((read = getline(&line, &len, fp)) != -1)
    {
		if(line == NULL || line[0] == '[')
			continue;

		memset(bus_name, 0, sizeof(bus_name));
		memset(bus_val, 0, sizeof(bus_val));

		sscanf(line, "%2s=\"%s", bus_name, bus_val);
		bus_val[strlen(bus_val)-1] = '\0';
		if(strcmp(bus_name, "hs") == 0)
		{
			trans_conf.hs = strdup(bus_val);
		}
		else if (strcmp(bus_name, "ls") == 0)
		{
			trans_conf.ls = strdup(bus_val);
		}
	}

	if(line != NULL)
		free(line);

	fclose(fp);
//	DBG_ERROR(LOG_PREFIX, "readTransBus end, hs:%s,ls:%s", trans_conf.hs, trans_conf.ls);
	return 0;
}

int CanSender::readJsonConfig()
{
    int fd_conf;
    char *filebuf = NULL;
	json_object *jobj = NULL;
	struct stat stbuf;

    fd_conf = open(STEERING_WHEEL_JSON, O_RDONLY);
	if(fd_conf < 0)
	{
		DBG_ERROR(LOG_PREFIX, "wheel configuration (steering_wheel.json) is not access");
		return -1;
	}

	FILE *fp = fdopen(fd_conf,"r");
	if(fp == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot read configuration file(steering_wheel.json)");
		return -1;
	}

	if(fstat(fd_conf, &stbuf) == -1)
	{
		DBG_ERROR(LOG_PREFIX, "can't get file state");
		return -1;
	}

	filebuf = (char *)malloc(stbuf.st_size);
	fread(filebuf, 1, stbuf.st_size, fp);
	fclose(fp);

	jobj = json_tokener_parse(filebuf);
	if(jobj == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "json: Invalid steering_wheel.json format");
		return -1;
	}
	json_object_object_foreach(jobj, key, val)
	{
		if(strcmp(key,"wheel_map") == 0)
		{
			wheel_define_init(json_object_get_string(val));
		}
		else if(strcmp(key,"gear_para") == 0)
		{
			wheel_gear_para_init(json_object_get_string(val));
		}
	}
	json_object_put(jobj);
	free(filebuf);

	return 0;
}

int CanSender::wheel_define_init(const char *fname)
{
	struct json_object *jobj;
	int fd_wheel_map;
	struct stat stbuf;
	char *filebuf;

	fd_wheel_map = open(fname, O_RDONLY);
	if(fd_wheel_map < 0)
	{
		DBG_ERROR(LOG_PREFIX, "wheel map is not access");
		return -1;
	}

	FILE *fp = fdopen(fd_wheel_map,"r");
	if(fp == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot read wheel map file");
		return -1;
	}

	if(fstat(fd_wheel_map, &stbuf) == -1)
	{
		DBG_ERROR(LOG_PREFIX, "cant get file state");
		return -1;
	}

	filebuf = (char *)malloc(stbuf.st_size);
	fread(filebuf, 1, stbuf.st_size, fp);
	fclose(fp);

	jobj = json_tokener_parse(filebuf);
	if(jobj == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot read data from file \"%s\"",fname);
		free(filebuf);
		return 1;
	}
	parse_json(jobj);
	json_object_put(jobj);

	free(filebuf);

	return 0;
}

int CanSender::wheel_gear_para_init(const char *fname)
{
	struct json_object *jobj;
	int fd_gear_para;
	struct stat stbuf;
	char *filebuf;

	fd_gear_para = open(fname, O_RDONLY);
	if(fd_gear_para < 0)
	{
		DBG_ERROR(LOG_PREFIX, "gear para is not access");
		return -1;
	}

	FILE *fp = fdopen(fd_gear_para,"r");
	if(fp == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot read gear para file");
		return -1;
	}

	if(fstat(fd_gear_para, &stbuf) == -1)
	{
		DBG_ERROR(LOG_PREFIX, "cant get file state.\n");
		return -1;
	}

	filebuf = (char*)malloc(stbuf.st_size);
	fread(filebuf, 1, stbuf.st_size, fp);
	fclose(fp);

	jobj = json_tokener_parse(filebuf);
	if(jobj == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "cannot data from file \"%s\"",fname);
		free(filebuf);
		return 1;
	}
	parse_gear_para_json(jobj);
	json_object_put(jobj);

	free(filebuf);

	return 0;
}

int CanSender::parse_json(json_object *obj)
{
	int err = 0;
    json_object_object_foreach(obj, key, val)
	{
        if (strcmp(key,"PROPERTYS") == 0)
        {
			err += parse_propertys(val);
		}
        else
        {
			++err;
			DBG_ERROR(LOG_PREFIX, "json: Unknown  key \"%s\"", key);
		}
    }
	return err;
}

int CanSender::parse_gear_para_json(json_object *obj)
{
	int err = 0;
	json_object * obj_speed_para;
	char *pos_name = NULL;
	double pos_val = 0;

    json_object_object_foreach(obj, key, val)
	{
        if (strcmp(key,"GEAR_PARA") == 0)
        {
        	enum json_type type = json_object_get_type(val);
			if (type == json_type_array)
			{
				int array_len = json_object_array_length(val);
				DBG_NOTICE(LOG_PREFIX, "array_len:%d!", array_len);
				for (int i = 0; i < array_len; i++)
				{
					obj_speed_para = json_object_array_get_idx(val, i);
					if(obj_speed_para)
					{
						json_object_object_foreach(obj_speed_para, key, val)
						{
							///
							if (strcmp("POS", key) == 0)
							{
								pos_name = (char *)json_object_get_string(val);
							}
							else if(strcmp("VAL", key) == 0)
							{
								pos_val = json_object_get_double(val);
							}

							///
							if(strcmp("First", pos_name) == 0)
							{
								gearRatio[1] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Second", pos_name) == 0)
							{
								gearRatio[2] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Third", pos_name) == 0)
							{
								gearRatio[3] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Fourth", pos_name) == 0)
							{
								gearRatio[4] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Fifth", pos_name) == 0)
							{
								gearRatio[5] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Sixth", pos_name) == 0)
							{
								gearRatio[6] = (double)(1.0 / pos_val);
							}
							else if(strcmp("Reverse", pos_name) == 0)
							{
								gearRatio[7] = (double)(1.0 / pos_val);
							}
						}
					}
				}
			}
			else
			{
				DBG_ERROR(LOG_PREFIX, "json: Need  array \"%s\"", key);
			}
		}
        else
        {
			DBG_ERROR(LOG_PREFIX, "json: Unknown  key \"%s\"", key);
		}
    }
	return err;
}

#define DEFAULT_PROP_CNT (16)
int CanSender::parse_propertys(json_object *obj_propertys)
{
	int err = 0;
	json_object * obj_property;
	size_t ms = sizeof(struct wheel_info_t) + (size_t)(DEFAULT_PROP_CNT*sizeof(struct prop_info_t));

	if(obj_propertys)
	{
		wheel_info = (struct wheel_info_t *)malloc(ms);
		if(wheel_info == NULL)
		{
			DBG_ERROR(LOG_PREFIX, "Not enogh memory");
			return 1;
		}
		memset(wheel_info, 0 , ms);

		enum json_type type = json_object_get_type(obj_propertys);
		if(type == json_type_array)
		{
			int array_len = json_object_array_length(obj_propertys);
			if(array_len > DEFAULT_PROP_CNT)
			{
				wheel_info = (struct wheel_info_t *)realloc(wheel_info, sizeof(struct wheel_info_t) + ((size_t)array_len * sizeof(struct prop_info_t)));
				if(wheel_info == NULL)
				{
					DBG_ERROR(LOG_PREFIX, "not enogh memory");
					exit(1);
				}

				memset(&wheel_info->property[DEFAULT_PROP_CNT], 0, (size_t)(array_len - DEFAULT_PROP_CNT)*sizeof(struct prop_info_t));
			}

			for(int i = 0; i < array_len; i++)
			{
				obj_property = json_object_array_get_idx(obj_propertys, i);
				err = parse_property(i, obj_property);
			}

			wheel_info->nData = (unsigned int)array_len;
		}
	}

	return err;
}

int CanSender::parse_property(int idx, json_object *obj_property)
{
	int var_type = 0;
	char *name = NULL;
	char *canid = NULL;

	if(obj_property)
	{
		json_object_object_foreach(obj_property, key, val)
		{
			if(strcmp("PROPERTY", key) == 0)
			{
				name = (char *)json_object_get_string(val);
			}
			else if(strcmp("TYPE", key) == 0)
			{
				const char *_varname = json_object_get_string(val);
				var_type = string2vartype(_varname);
			}
			else if(strcmp("CANID", key) == 0)
			{
				canid = (char *)json_object_get_string(val);
			}
			else if(strcmp("BIT_POSITION", key) == 0)
			{
				const char * tmp = json_object_get_string(val);
				wheel_info->property[idx].bit_pos = (uint8_t)strtoul(tmp, 0, 0);
			}
			else if(strcmp("BIT_SIZE", key) == 0)
			{
				const char * tmp = json_object_get_string(val);
				wheel_info->property[idx].bit_size = (uint8_t)strtoul(tmp, 0, 0);
			}
			else if(strcmp("DLC", key) == 0)
			{
				const char * tmp = json_object_get_string(val);
				wheel_info->property[idx].dlc = (uint8_t)strtoul(tmp, 0, 0);
			}
		}

		wheel_info->property[idx].name = strdup(name);
		wheel_info->property[idx].var_type = (unsigned char)var_type;
		wheel_info->property[idx].can_id = strdup(canid);
	}

	return 0;
}

void CanSender::updateValue(const char *prop, int val)
{
	// DBG_INFO(LOG_PREFIX, "updateValue");
	unsigned int nProp = wheel_info->nData;
	for(unsigned int i = 0; i < nProp; i++)
	{
		if(strcmp(prop, wheel_info->property[i].name) == 0)
		{
			if(wheel_info->property[i].curValue.int16_val != val)
			{
				wheel_info->property[i].curValue.int16_val = (int16_t)val;

				// DBG_INFO(LOG_PREFIX, "notify_property_changed name=%s,value=%d", 
				// 	wheel_info->property[i].name, wheel_info->property[i].curValue);
				int rc = carla::push(makeCanData(&wheel_info->property[i]));
				if(rc < 0)
				{
					DBG_ERROR(LOG_PREFIX, "push failed");
				}
			}
		}
	}
}

} // namespace carla
