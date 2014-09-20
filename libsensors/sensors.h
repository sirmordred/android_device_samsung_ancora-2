/*
 * Copyright (C) 2008 The Android Open Source Project
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

#ifndef ANDROID_SENSORS_H
#define ANDROID_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

/*****************************************************************************/

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

#define ID_A (0)
#define ID_M (1)
#define ID_OR (2)
#define ID_RV (3)
#define ID_P (4)

#define AKM_DEVICE_NAME "/dev/akm8975"

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/* the GP2A is a binary proximity sensor that triggers around 5 cm on
 * this hardware */
#define PROXIMITY_THRESHOLD_GP2A  5.0f

/*****************************************************************************/

/* For Accelerometer */
#define EVENT_TYPE_ACCEL_X          ABS_X
#define EVENT_TYPE_ACCEL_Y          ABS_Y
#define EVENT_TYPE_ACCEL_Z          ABS_Z

/* For Magnetometer */
#define EVENT_TYPE_MAGV_X           ABS_RX
#define EVENT_TYPE_MAGV_Y           ABS_RY
#define EVENT_TYPE_MAGV_Z           ABS_RZ

/* For Orientation */
#define EVENT_TYPE_YAW              ABS_HAT0Y
#define EVENT_TYPE_PITCH            ABS_HAT1X
#define EVENT_TYPE_ROLL             ABS_HAT1Y

/* For Rotation Vector */
#define EVENT_TYPE_ROTVEC_X         ABS_TILT_X
#define EVENT_TYPE_ROTVEC_Y         ABS_TILT_Y
#define EVENT_TYPE_ROTVEC_Z         ABS_TOOL_WIDTH
#define EVENT_TYPE_ROTVEC_W	    ABS_VOLUME

#define EVENT_TYPE_PROXIMITY        ABS_DISTANCE

// conversion of acceleration data to SI units (m/s^2)
#define RANGE_A                     (2*GRAVITY_EARTH)
#define RESOLUTION_A                (RANGE_A/(512))
#define CONVERT_A                   (1.0f/1000000.0f)
#define CONVERT_A_X                 (CONVERT_A)
#define CONVERT_A_Y                 (CONVERT_A)
#define CONVERT_A_Z                 (CONVERT_A)

/* conversion of magnetic data to uT units */
#define CONVERT_M		    (0.06f)

/* conversion of orientation data (Q6) to degree units */
#define CONVERT_OR		    (1.0f / 64.0f)
/* conversion of rotation vector (Q14) data to float */
#define CONVERT_RV		    CONVERT_Q14

/*****************************************************************************/

__END_DECLS

#endif  // ANDROID_SENSORS_H
