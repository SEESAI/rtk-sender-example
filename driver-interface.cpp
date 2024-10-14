#include "driver-interface.h"
#include <iostream>

int DriverInterface::callback_entry(GPSCallbackType type, void* data1, int data2, void* user)
{
    auto self = reinterpret_cast<DriverInterface*>(user);
    return self->callback(type, data1, data2);
}

int DriverInterface::callback(GPSCallbackType type, void* data1, int data2)
{
    switch (type) {
        case GPSCallbackType::readDeviceData: {
            if (serial_comms_.bytesAvailable() == 0) {
                int timeout = *((int *) data1);
                if (!serial_comms_.waitForReadyRead(timeout))
                    return 0;
            }
            return (int)serial_comms_.read((char*)data1, data2);
        }
        case GPSCallbackType::writeDeviceData:
            if (serial_comms_.write((char*) data1, data2) >= 0) {
                if (serial_comms_.waitForBytesWritten(-1))
                    return data2;
            }
            return -1;
        case GPSCallbackType::setBaudrate:
            return serial_comms_.setBaudRate(data2) ? 0 : -1;

        case GPSCallbackType::gotRTCMMessage:
            send_rtcm_data((uint8_t*)data1, data2);
            return 0;
	case GPSCallbackType::surveyInStatus:
	{
	    SurveyInStatus* status = (SurveyInStatus*)data1;
	    printf("Position: %f %f %f\n", status->latitude, status->longitude, status->altitude);
	    printf("Survey-In status: %lu s, cur accuracy: %lu mm, valid: %d, active: %d\n", status->duration, status->mean_accuracy, (int)(status->flags & 1), (int)((status->flags >> 1) & 1));
	    return 0;
	}
        default:
            // Ignore rest.
            return 0;
    }
}

void DriverInterface::send_rtcm_data(const uint8_t* data, int data_len)
{
    if (!rtk_plugin_) {
        if (mavsdk_.systems().empty()) {
            printf("No mavlink system available yet\n");
            return;
        }

        rtk_plugin_ = std::make_shared<mavsdk::Rtk>(mavsdk_.systems()[0]);
        telemetry_plugin_ = std::make_shared<mavsdk::Telemetry>(mavsdk_.systems()[0]);
    }

    mavsdk::Rtk::RtcmData rtcm_data;
    rtcm_data.data.insert(rtcm_data.data.end(), data, data + data_len);
    rtk_plugin_->send_rtcm_data(rtcm_data);

    // std::cout << "Fix type: " << telemetry_plugin_->gps_info().fix_type << '\n';
}
