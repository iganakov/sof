/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2023 Intel Corporation
 *
 * Author: Ievgen Ganakov <ievgen.ganakov@intel.com>
 */

#ifndef __SOF_LIB_AMS_MSG_H__
#define __SOF_LIB_AMS_MSG_H__

/* AMS messages */

/* Key-phrase detected AMS message uuid: 80a11122-b36c-11ed-afa1-0242ac120002*/
#define AMS_KPD_MSG_UUID 0x80a11122, 0xb36c, 0x11ed, 0xaf, 0xa1, 0x02, 0x42, 0xac, 0x12, 0x00, 0x02

#define DEFINE_UUID_(name, l1, s1, s2, b1, b2, b3, b4, b5, b6, b7, b8) \
	static const uint32_t name[] = \
	{ \
		l1, \
		s2 << 16 | s1, \
		b4 << 24 | b3 << 16 | b2 << 8 | b1, \
		b8 << 24 | b7 << 16 | b6 << 8 | b5, \
	}

#define DEFINE_UUID(name, ...) DEFINE_UUID_(name, __VA_ARGS__)

#endif /* __SOF_LIB_AMS_MSG_H__ */
