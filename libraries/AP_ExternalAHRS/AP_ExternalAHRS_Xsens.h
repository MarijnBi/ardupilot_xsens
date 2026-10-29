/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected Xsens MTi GNSS/INS devices (Xbus protocol, MTData2)
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_Common/time.h>

class AP_ExternalAHRS_Xsens : public AP_ExternalAHRS_backend
{
public:
    AP_ExternalAHRS_Xsens(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    const char* get_name() const override { return "Xsens"; }

    // all work is done in update_thread()
    void update() override {}

    uint8_t num_gps_sensors(void) const override { return 1; }

private:
    // one decoded MTData2 message
    struct Sample {
        uint32_t status_word;
        float temperature;
        Quaternion quat;
        Vector3f accel;
        Vector3f gyro;
        Vector3f mag;
        float pressure_pa;
        double lat, lon, alt_ellipsoid;
        Vector3f vel_ned;
        struct tm utc;
        uint32_t utc_ns;
        uint16_t have;      // bitmask of HAVE_*
    };

    // the parts of the GNSS PVT data (XDI 0x7010) that are used
    struct GnssPvt {
        uint8_t fix_type;
        uint8_t flags;
        uint8_t num_sv;
        int32_t height_mm;      // above ellipsoid
        int32_t height_msl_mm;
        uint32_t h_acc_mm, v_acc_mm, s_acc_mms;
        uint16_t vdop, hdop;    // scaled by 100
    };

    void update_thread();
    void check_uart();
    bool parse_mtdata2(const uint8_t *payload, uint16_t len, Sample &s, GnssPvt &gnss, bool &got_pvt) const;
    void handle_sample(const Sample &s);
    void post_gps(const Sample &s, const Location &loc);

    bool filter_valid() const;
    bool filter_gnss_aided() const;
    bool gnss_valid(uint32_t now_ms) const;
    AP_GPS_FixType fix_type() const;

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;

    // receive buffer
    uint8_t buf[1024];
    uint16_t buf_len;

    uint32_t last_pkt_ms;       // last MTData2 message
    uint32_t last_pos_ms;       // last MTData2 message with a position
    uint32_t last_pvt_ms;       // last GNSS PVT data
    uint32_t last_gps_post_ms;

    // last time each MTData2 item (Sample::have bit) was received, for healthy() and
    // the pre-arm check of the MT Manager output configuration
    static constexpr uint8_t NUM_ITEMS = 12;
    uint32_t item_ms[NUM_ITEMS];
    bool item_recent(uint16_t have, uint32_t now_ms, uint32_t timeout_ms) const;

    GnssPvt pvt;
    bool have_pvt;
    float geoid_separation_m;   // ellipsoid height - AMSL height
    bool have_geoid_separation;
    uint32_t status_word;
    bool have_status_word;
};

#endif  // AP_EXTERNAL_AHRS_XSENS_ENABLED
