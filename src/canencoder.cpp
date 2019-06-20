/*
 * Copyright (c) 2017-2018 TOYOTA MOTOR CORPORATION
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
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <pthread.h>
#include <search.h>

#include "canencoder.hpp"
#include "debugmsg.hpp"

namespace carla
{

#define CANID_DELIM '#'
#define DATA_SEPERATOR '.'
#define ENABLE1_TYPENAME "ENABLE-1"	/* spec. type1.json original type name */

static struct can_data_t *phead = NULL, *ptail = NULL;
static pthread_mutex_t lock;
static char buf[MAX_CANDATA_SIZE+1] = {0};
static char str[MAX_LENGTH+1] = {0};
static void *canmsg_root = NULL;


/* CAN DLC to real data length conversion helpers */
static const unsigned char dlc2len[] = {0, 1, 2, 3, 4, 5, 6, 7,
					8, 12, 16, 20, 24, 32, 48, 64};

/*
 * get data length from can_dlc with sanitized can_dlc
 */
unsigned char can_dlc2len(unsigned char can_dlc)
{
	return dlc2len[can_dlc & 0x0F];
}

static const unsigned char len2dlc[] = {0, 1, 2, 3, 4, 5, 6, 7, 8,		/* 0 - 8 */
					9, 9, 9, 9,											/* 9 - 12 */
					10, 10, 10, 10,										/* 13 - 16 */
					11, 11, 11, 11,										/* 17 - 20 */
					12, 12, 12, 12,										/* 21 - 24 */
					13, 13, 13, 13, 13, 13, 13, 13,						/* 25 - 32 */
					14, 14, 14, 14, 14, 14, 14, 14,						/* 33 - 40 */
					14, 14, 14, 14, 14, 14, 14, 14,						/* 41 - 48 */
					15, 15, 15, 15, 15, 15, 15, 15,						/* 49 - 56 */
					15, 15, 15, 15, 15, 15, 15, 15};					/* 57 - 64 */

const char *vartype_strings[] = {
/* 0 */	"void",
/* 1 */	"int8_t",
/* 2 */	"int16_t",
/* 3 */	"int",
/* 4 */	"int32_t",
/* 5 */	"int64_t",
/* 6 */	"uint8_t",
/* 7 */	"uint16_t",
/* 8 */	"uint",
/* 9 */	"uint32_t",
/* 10 */"uint64_t",
/* 11 */	"string",
/* 12 */	"bool",
/* 13 */	"LIST",
/* 14 */ ENABLE1_TYPENAME
};

/*
 * map the sanitized data length to an appropriate data length code
 */
unsigned char can_len2dlc(unsigned char len)
{
	if (len > 64)
		return 0xF;

	return len2dlc[len];
}

/*
 * asc to nibble
 */
unsigned char asc2nibble(char c)
{

	if ((c >= '0') && (c <= '9'))
		return c - '0';

	if ((c >= 'A') && (c <= 'F'))
		return c - 'A' + 10;

	if ((c >= 'a') && (c <= 'f'))
		return c - 'a' + 10;

	return 16; /* error */
}

static int canmsg_compare(const void *pa, const void *pb)
{
	struct canmsg_info_t *a =(struct canmsg_info_t *)pa;
	struct canmsg_info_t *b =(struct canmsg_info_t *)pb;
	return strcmp(a->canid,b->canid);
}

/*
 * init
 */
void init_can_encoder(void)
{
	pthread_mutex_init(&lock, NULL);
	phead = NULL;
	ptail = NULL;
}

/*
 * push can msg to queue
 */
int push(char *dat)
{
	if (dat == NULL)
	{
		DBG_ERROR(LOG_PREFIX, "push data is NULL");
		return -1;
	}

	struct can_data_t *p = (struct can_data_t *)malloc(sizeof(struct can_data_t));
	strncpy(p->dat, dat, MAX_CANDATA_SIZE);
	p->dat[MAX_CANDATA_SIZE] = '\0';
	p->next = NULL;

	pthread_mutex_lock(&lock);
	if (ptail == NULL)
	{
		ptail = p;
		phead = p;
	}
	else
	{
		ptail->next = p;
		ptail = p;
	}
	pthread_mutex_unlock(&lock);
	return 0;
}

/*
 * pop can msg from queue
 */
struct can_data_t* pop(void)
{
	struct can_data_t *p = NULL;
	pthread_mutex_lock(&lock);
	if (phead != NULL)
	{
		p = phead;
		if (phead->next != NULL)
		{
			phead = p->next;
		}
		else
		{
			phead = NULL;
			ptail = NULL;
		}
	}
	pthread_mutex_unlock(&lock);
	return p;
}

/*
 * clear transmission msg queue
 */
void clear(void)
{
	struct can_data_t *p = NULL;
	pthread_mutex_lock(&lock);
	while (phead != NULL)
	{
		p = phead;
		phead = phead->next;
		free(p);
	}
	ptail = NULL;
	pthread_mutex_unlock(&lock);
}

/*
 * make "0" string
 */
static char *makeZeroString(uint8_t dlc)
{
	int len = dlc * 2;

	if (len > MAX_LENGTH)
	{
		DBG_ERROR(LOG_PREFIX, "makeZeroString input dlc error; dlc=%d",dlc);
		return NULL;
	}

	for (int i = 0; i < len; i++) {
		str[i] = '0';
	}

	str[len + 1] = '\0';

	return str;
}

static struct canmsg_info_t * getCanMsg_dict(const char * can_id)
{
	struct canmsg_info_t info;
	strncpy(info.canid, can_id, sizeof(info.canid));
	info.value = 0;

	void *v;
	v = tfind((void *)&info, &canmsg_root, canmsg_compare);
	if (v == NULL)
	{
		/* new msg, add node */
		struct canmsg_info_t * p = (struct canmsg_info_t *)malloc(sizeof(struct canmsg_info_t));
		strncpy(p->canid, can_id, sizeof(p->canid));
		p->value = 0;
		v = tsearch((void *)p, &canmsg_root, canmsg_compare);
		if (v == NULL)
		{
			DBG_ERROR(LOG_PREFIX, "add canmsg failed: not enogh memory?");
		}
	}

	return (*(struct canmsg_info_t **)v);
}

/*
 * make can frame data
 */
char * makeCanData(struct prop_info_t *property_info)
{
	char tmp[MAX_LENGTH+1] = {0};
	u_int64_t val = 0, mask = 0;
	struct canmsg_info_t * p = getCanMsg_dict(property_info->can_id);
	if (p == NULL)
	{
		return NULL;
	}

	memset(buf, 0, sizeof(buf));
	sprintf(buf, "%s#%s", property_info->can_id, makeZeroString(property_info->dlc));
	mask = (1 << (property_info->bit_size)) - 1;
	val = mask & property_info->curValue.uint32_val;
	val = val << ((property_info->dlc * 8) - property_info->bit_size - property_info->bit_pos);
	mask = mask << ((property_info->dlc * 8) - property_info->bit_size - property_info->bit_pos);
	mask = ~mask;
	p->value = p->value & mask;
	p->value = p->value | val;

	sprintf(tmp, "%lx", p->value);
	strncpy((buf + 4 + property_info->dlc * 2 -strlen(tmp)), tmp ,strlen(tmp));

//	DBG_ERROR(LOG_PREFIX, "makeCanData buf is [%s]", buf);

	return buf;
}

/*
 * convert to canframe format
 */
int parse_canframe(char *cs, struct canfd_frame *cf)
{
	/* documentation see lib.h */

	int i, idx, dlen, len;
	int maxdlen = CAN_MAX_DLEN;
	int ret = CAN_MTU;
	unsigned char tmp;

	len = strlen(cs);

	memset(cf, 0, sizeof(*cf)); /* init CAN FD frame, e.g. LEN = 0 */

	if (len < 4)
		return 0;

	if (cs[3] == CANID_DELIM) { /* 3 digits */

		idx = 4;
		for (i=0; i<3; i++){
			if ((tmp = asc2nibble(cs[i])) > 0x0F)
				return 0;
			cf->can_id |= (tmp << (2-i)*4);
		}

	} else if (cs[8] == CANID_DELIM) { /* 8 digits */

		idx = 9;
		for (i=0; i<8; i++){
			if ((tmp = asc2nibble(cs[i])) > 0x0F)
				return 0;
			cf->can_id |= (tmp << (7-i)*4);
		}
		if (!(cf->can_id & CAN_ERR_FLAG)) /* 8 digits but no errorframe?  */
			cf->can_id |= CAN_EFF_FLAG;   /* then it is an extended frame */

	} else
		return 0;

	if((cs[idx] == 'R') || (cs[idx] == 'r')){ /* RTR frame */
		cf->can_id |= CAN_RTR_FLAG;

		/* check for optional DLC value for CAN 2.0B frames */
		if(cs[++idx] && (tmp = asc2nibble(cs[idx])) <= CAN_MAX_DLC)
			cf->len = tmp;

		return ret;
	}

	if (cs[idx] == CANID_DELIM) { /* CAN FD frame escape char '##' */

		maxdlen = CANFD_MAX_DLEN;
		ret = CANFD_MTU;

		/* CAN FD frame <canid>##<flags><data>* */
		if ((tmp = asc2nibble(cs[idx+1])) > 0x0F)
			return 0;

		cf->flags = tmp;
		idx += 2;
	}

	for (i=0, dlen=0; i < maxdlen; i++){

		if(cs[idx] == DATA_SEPERATOR) /* skip (optional) separator */
			idx++;

		if(idx >= len) /* end of string => end of data */
			break;

		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cf->data[i] = (tmp << 4);
		if ((tmp = asc2nibble(cs[idx++])) > 0x0F)
			return 0;
		cf->data[i] |= tmp;
		dlen++;
	}
	cf->len = dlen;

	return ret;
}

int string2vartype(const char *varname)
{
	unsigned int i;
	if (varname == NULL)
	{
		return -1;
	}

	for(i=0; i < (sizeof(vartype_strings)/sizeof(char *)); i++)
	{
		if(strcmp(varname, vartype_strings[i]) == 0)
			return (int)i;
	}

	return -1;
};

} // namespace carla
