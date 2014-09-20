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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <dlfcn.h>

#include <cutils/log.h>
#include "AkmSensor.h"

#define LOGTAG "AkmSensor"

//#define ALOG_NDEBUG 0

/*****************************************************************************/

int (*akm_is_sensor_enabled)(uint32_t sensor_type);
int (*akm_enable_sensor)(uint32_t sensor_type);
int (*akm_disable_sensor)(uint32_t sensor_type);
int (*akm_set_delay)(uint32_t sensor_type, uint64_t delay);

int stub_is_sensor_enabled(uint32_t sensor_type) {
    return 0;
}

int stub_enable_disable_sensor(uint32_t sensor_type) {
    return -ENODEV;
}

int stub_set_delay(uint32_t sensor_type, uint64_t delay) {
    return -ENODEV;
}


AkmSensor::AkmSensor()
: SensorBase(NULL, NULL),
      mEnabled(0),
      mPendingMask(0),
      mInputReader(32)
{
    /* Open the library before opening the input device.  The library
     * creates a uinput device.
     */
    if (loadAKMLibrary() == 0) {
        data_name = "compass_sensor";
        data_fd = openInput("compass_sensor");
    }

    //Incase first time fails
    if(data_fd < 0){
         ALOGI("%s: retrying to open compass sensor", LOGTAG);
         data_fd = openInput("compass_sensor");
    }

    if(data_fd > 0){
         ALOGI("%s: compass sensor successfully opened: %i", LOGTAG, data_fd);
    }else{
         ALOGI("%s: failed to open compass sensor", LOGTAG);
    }

    memset(mPendingEvents, 0, sizeof(mPendingEvents));

    mPendingEvents[MagneticField].version = sizeof(sensors_event_t);
    mPendingEvents[MagneticField].sensor = ID_M;
    mPendingEvents[MagneticField].type = SENSOR_TYPE_MAGNETIC_FIELD;
    mPendingEvents[MagneticField].magnetic.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[Orientation  ].version = sizeof(sensors_event_t);
    mPendingEvents[Orientation  ].sensor = ID_OR;
    mPendingEvents[Orientation  ].type = SENSOR_TYPE_ORIENTATION;
    mPendingEvents[Orientation  ].orientation.status = SENSOR_STATUS_ACCURACY_HIGH;

    mPendingEvents[RotationVector].version = sizeof(sensors_event_t);
    mPendingEvents[RotationVector].sensor = ID_RV;
    mPendingEvents[RotationVector].type = SENSOR_TYPE_ROTATION_VECTOR;

    // read the actual value of all sensors if they're enabled already
    struct input_absinfo absinfo;
    short flags = 0;
    if (akm_is_sensor_enabled(SENSOR_TYPE_MAGNETIC_FIELD))  {
        mEnabled |= 1<<MagneticField;
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_MAGV_X), &absinfo)) {
	    mPendingEvents[MagneticField].magnetic.x = value * CONVERT_M;
	}
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_MAGV_Y), &absinfo)) {
	    mPendingEvents[MagneticField].magnetic.y = value * CONVERT_M;
	}
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_MAGV_Z), &absinfo)) {
	    mPendingEvents[MagneticField].magnetic.z = value * CONVERT_M;
	}
    }
}
    if (akm_is_sensor_enabled(SENSOR_TYPE_ORIENTATION))
        mEnabled |= 1<<Orientation;
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_YAW), &absinfo)) {
	    mPendingEvents[Orientation].orientation.azimuth = value * CONVERT_OR;
	}
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_PITCH), &absinfo)) {
	    mPendingEvents[Orientation].orientation.pitch = value * CONVERT_OR;
        }
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ROLL), &absinfo)) {
	    mPendingEvents[Orientation].orientation.roll = value * CONVERT_OR;
	}
    }
}
    if (akm_is_sensor_enabled(SENSOR_TYPE_ROTATION_VECTOR))
        mEnabled |= 1<<RotationVector;
        if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ROTVEC_X), &absinfo)) {
	    mPendingEvents[RotationVector].data[0] = value * CONVERT_RV;
        }
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ROTVEC_Y), &absinfo)) {
	    mPendingEvents[RotationVector].data[1] = value * CONVERT_RV;
        }
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ROTVEC_Z), &absinfo)) {
	    mPendingEvents[RotationVector].data[2] = value * CONVERT_RV;
        }
	if  (!ioctl(data_fd, EVIOCGABS(EVENT_TYPE_ROTVEC_W), &absinfo)) {
	    mPendingEvents[RotationVector].data[3] = value * CONVERT_RV;
	}
}

AkmSensor::~AkmSensor()
{
    if (mLibAKM) {
        unsigned ref = ::dlclose(mLibAKM);
    }
}


int AkmSensor::setInitialState()
{
    return 0;
}

int AkmSensor::enable(int32_t handle, int en)
{
    int what = -1;

    switch (handle) {
        case ID_M: what = MagneticField; break;
        case ID_OR: what = Orientation;   break;
        case ID_RV: what = RotationVector;  break;
    }

    if (uint32_t(what) >= numSensors)
        return -EINVAL;

    int newState  = en ? 1 : 0;
    int err = 0;

    if ((uint32_t(newState)<<what) != (mEnabled & (1<<what))) {

        uint32_t sensor_type;

        switch (what) {
            case MagneticField: sensor_type = SENSOR_TYPE_MAGNETIC_FIELD; break;
            case Orientation:   sensor_type = SENSOR_TYPE_ORIENTATION;  break;
            case RotationVector: sensor_type = SENSOR_TYPE_ROTATION_VECTOR; break;
        }
        short flags = newState;
        if (en){
            err = akm_enable_sensor(sensor_type);
        }else{
            err = akm_disable_sensor(sensor_type);
        }

        err = sspEnable(LOGTAG, SSP_MAG, en);
        setInitialState();
        setDelay(handle, 66667000); //set an initial delay after enabling

        ALOGE_IF(err, "Could not change sensor state (%s)", strerror(-err));
        if (!err) {
            mEnabled &= ~(1<<what);
            mEnabled |= (uint32_t(flags)<<what);
        }
    }
    return err;
}

int AkmSensor::setDelay(int32_t handle, int64_t ns)
{
    int what = -1;
    int fd;
    uint32_t sensor_type = 0;

    if (ns < 0)
        return -EINVAL;

    switch (handle) {
        case ID_M: sensor_type = SENSOR_TYPE_MAGNETIC_FIELD; break;
        case ID_OR: sensor_type = SENSOR_TYPE_ORIENTATION; break;
        case ID_RV: sensor_type = SENSOR_TYPE_ROTATION_VECTOR; break;
    }

    if (sensor_type == 0)
        return -EINVAL;

    mDelays[what] = ns;
    return 0;
}

int AkmSensor::loadAKMLibrary()
{
    mLibAKM = dlopen("libakm.so", RTLD_NOW);

    if (!mLibAKM) {
        akm_is_sensor_enabled = stub_is_sensor_enabled;
        akm_enable_sensor = stub_enable_disable_sensor;
        akm_disable_sensor = stub_enable_disable_sensor;
        akm_set_delay = stub_set_delay;
        ALOGE("%s: unable to load AKM Library, %s", LOGTAG, dlerror());
        return -ENOENT;
    }

    *(void **)&akm_is_sensor_enabled = dlsym(mLibAKM, "akm_is_sensor_enabled");
    *(void **)&akm_enable_sensor = dlsym(mLibAKM, "akm_enable_sensor");
    *(void **)&akm_disable_sensor = dlsym(mLibAKM, "akm_disable_sensor");
    *(void **)&akm_set_delay = dlsym(mLibAKM, "akm_set_delay");

    return 0;
}

int AkmSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_REL) {
            processEvent(event->code, event->value);
            mInputReader.next();
        } else if (type == EV_ABS) {
            processEvent(event->code, event->value);
            mInputReader.next();
        } else if (type == EV_SYN) {
            int64_t time = timevalToNano(event->time);
            for (int j=0 ; count && mPendingMask && j<numSensors ; j++) {
                if (mPendingMask & (1<<j)) {
                    mPendingMask &= ~(1<<j);
                    mPendingEvents[j].timestamp = time;
                    if (mEnabled & (1<<j)) {
                        *data++ = mPendingEvents[j];
                        count--;
                        numEventReceived++;
                    }
                }
            }
            if (!mPendingMask) {
                mInputReader.next();
            }
        } else {
            ALOGE("%s: unknown event (type=%d, code=%d)", LOGTAG,
                    type, event->code);
            mInputReader.next();
        }
    }
    return numEventReceived;
}

void AkmSensor::processEvent(int code, int value)
{
    switch (code) {
        case EVENT_TYPE_MAGV_X:
            mPendingMask |= 1<<MagneticField;
            mPendingEvents[MagneticField].magnetic.x = value * CONVERT_M;
            break;
        case EVENT_TYPE_MAGV_Y:
            mPendingMask |= 1<<MagneticField;
            mPendingEvents[MagneticField].magnetic.y = value * CONVERT_M;
            break;
        case EVENT_TYPE_MAGV_Z:
            mPendingMask |= 1<<MagneticField;
            mPendingEvents[MagneticField].magnetic.z = value * CONVERT_M;
            break;

	case EVENT_TYPE_YAW:
	    mPendingMask |= 1<<Orientation;
	    mPendingEvents[Orientation].orientation.azimuth = value * CONVERT_OR;
	    break;
	case EVENT_TYPE_PITCH:
	    mPendingMask |= 1<<Orientation;
	    mPendingEvents[Orientation].orientation.pitch = value * CONVERT_OR;
	    break;
	case EVENT_TYPE_ROLL:
	    mPendingMask |= 1<<Orientation;
	    mPendingEvents[Orientation].orientation.roll = value * CONVERT_OR;
	    break;

	case EVENT_TYPE_ROTVEC_X:
	    mPendingMask |= 1<<RotationVector;
	    mPendingEvents[RotationVector].data[0] = value * CONVERT_RV;
	    break;
	case EVENT_TYPE_ROTVEC_Y:
	    mPendingMask |= 1<<RotationVector;
	    mPendingEvents[RotationVector].data[1] = value * CONVERT_RV;
	    break;
	case EVENT_TYPE_ROTVEC_Z:
	    mPendingMask |= 1<<RotationVector;
	    mPendingEvents[RotationVector].data[2] = value * CONVERT_RV;
	    break;
	case EVENT_TYPE_ROTVEC_W:
	    mPendingMask |= 1<<RotationVector;
	    mPendingEvents[RotationVector].data[3] = value * CONVERT_RV;
	    break;
    }
}
