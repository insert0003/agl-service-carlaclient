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

#ifndef TMCAGL_CAN_ENCODER_HPP
#define TMCAGL_CAN_ENCODER_HPP

#include <string.h>
#include <stdint.h>
#include <sys/types.h>

#include <sys/socket.h> /* for sa_family_t */
#include <linux/can.h>
#include <linux/can/error.h>

namespace carla
{

#define MAX_LENGTH	16
#define MAX_CANDATA_SIZE 20

struct can_data_t
{
	char dat[MAX_CANDATA_SIZE+1];
	struct can_data_t * next;
};

struct canmsg_info_t
{
	char canid[4];
	u_int64_t value;
};

enum var_type_t
{
/* 0 */	VOID_T,
/* 1 */	INT8_T,
/* 2 */	INT16_T,
/* 3 */	INT_T,
/* 4 */	INT32_T,
/* 5 */	INT64_T,
/* 6 */	UINT8_T,
/* 7 */	UINT16_T,
/* 8 */	UINT_T,
/* 9 */	UINT32_T,
/* 10 */	UINT64_T,
/* 11 */	STRING_T,
/* 12 */	BOOL_T,
/* 13 */	ARRAY_T,
/* 14 */	ENABLE1_T	/* AMB#CANRawPlugin original type */
};

union data_content_t
{
	uint32_t uint32_val;
	int32_t  int32_val;
	uint16_t uint16_val;
	int16_t  int16_val;
	uint8_t  uint8_val;
	int8_t   int8_val;
	int8_t     bool_val;
};

struct prop_info_t
{
	const char * name;
	unsigned char var_type;
	const char * can_id;
	uint8_t bit_pos;
	uint8_t bit_size;
	uint8_t dlc;
	union data_content_t curValue;

	struct prop_info_t *next;
};


extern void init_can_encoder(void);
extern int push(char *dat);
extern struct can_data_t* pop(void);
extern void clear(void);
extern char * makeCanData(struct prop_info_t *property_info);
extern unsigned char can_dlc2len(unsigned char can_dlc);
extern unsigned char can_len2dlc(unsigned char len);
extern int parse_canframe(char *cs, struct canfd_frame *cf);
extern int string2vartype(const char *varname);

} // namespace carla

#endif /* TMCAGL_CAN_ENCODER_HPP */
