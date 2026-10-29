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
  simulate a serial Xsens MTi GNSS/INS
*/

#include "SIM_Xsens.h"

#include <SITL/SITL.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_HAL/utility/sparse-endian.h>
#include "SIM_GPS.h"

using namespace SITL;

static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
static constexpr uint8_t XBUS_MASTERDEVICE = 0xFF;

Xsens::Xsens() : SerialDevice::SerialDevice()
{
}

void Xsens::send_message(uint8_t msg_id, const uint8_t *payload, uint16_t len)
{
    uint8_t msg[5 + 254];
    if (len > 254) {
        return;
    }
    msg[0] = XBUS_PREAMBLE;
    msg[1] = XBUS_MASTERDEVICE;
    msg[2] = msg_id;
    msg[3] = len;
    memcpy(&msg[4], payload, len);
    uint8_t sum = 0;
    for (uint16_t i = 1; i < 4 + len; i++) {
        sum += msg[i];
    }
    msg[4 + len] = -sum;
    write_to_autopilot((const char *)msg, 5 + len);
}

// UTC time of the simulation
static void simulation_timeval(struct timeval *tv)
{
    const uint64_t now_us = AP_HAL::micros64();
    tv->tv_sec = AP::sitl()->start_time_UTC + now_us / 1000000ULL;
    tv->tv_usec = now_us % 1000000ULL;
}

// helpers to append a MTData2 data item
static uint8_t *put_xdi(uint8_t *p, uint16_t xdi, uint8_t size)
{
    put_be16_ptr(p, xdi);
    p[2] = size;
    return p + 3;
}

static uint8_t *put_float(uint8_t *p, float v)
{
    uint32_t u;
    memcpy(&u, &v, 4);
    put_be32_ptr(p, u);
    return p + 4;
}

static uint8_t *put_fp1632(uint8_t *p, double v)
{
    const int64_t fp = int64_t(v * 4294967296.0);
    put_be32_ptr(p, uint32_t(fp & 0xFFFFFFFF));
    put_be16_ptr(p+4, uint16_t(int16_t(fp >> 32)));
    return p + 6;
}

static uint8_t *put_vector3f(uint8_t *p, const Vector3f &v)
{
    p = put_float(p, v.x);
    p = put_float(p, v.y);
    return put_float(p, v.z);
}

void Xsens::send_mtdata2(bool with_pvt)
{
    const auto &fdm = _sitl->state;
    uint8_t payload[254] {};
    uint8_t *p = payload;

    struct timeval tv;
    simulation_timeval(&tv);
    struct tm tmd {};
    const time_t t = tv.tv_sec;
    const struct tm *utc = gmtime_r(&t, &tmd);

    p = put_xdi(p, 0x1010, 12);     // UTC time
    put_be32_ptr(p, tv.tv_usec * 1000);
    put_be16_ptr(p+4, utc->tm_year + 1900);
    p[6] = utc->tm_mon + 1;
    p[7] = utc->tm_mday;
    p[8] = utc->tm_hour;
    p[9] = utc->tm_min;
    p[10] = utc->tm_sec;
    p[11] = 0x07;   // valid
    p += 12;

    float pressure, temp_K;
    AP_Baro::get_pressure_temperature_for_alt_amsl(fdm.altitude, pressure, temp_K);

    p = put_xdi(p, 0x0810, 4);      // temperature
    p = put_float(p, KELVIN_TO_C(temp_K));

    p = put_xdi(p, 0x2010, 16);     // quaternion, NED
    p = put_float(p, fdm.quaternion.q1);
    p = put_float(p, fdm.quaternion.q2);
    p = put_float(p, fdm.quaternion.q3);
    p = put_float(p, fdm.quaternion.q4);

    p = put_xdi(p, 0x4020, 12);     // acceleration, FRD
    p = put_vector3f(p, Vector3f(fdm.xAccel, fdm.yAccel, fdm.zAccel));

    p = put_xdi(p, 0x8020, 12);     // rate of turn, FRD
    p = put_vector3f(p, Vector3f(radians(fdm.rollRate), radians(fdm.pitchRate), radians(fdm.yawRate)));

    p = put_xdi(p, 0xC020, 12);     // magnetic field, FRD, arbitrary units
    p = put_vector3f(p, fdm.bodyMagField / 490.0f);

    p = put_xdi(p, 0xE020, 4);      // status word: filter valid, GNSS fix, filter mode with GNSS
    put_be32_ptr(p, (1U<<1) | (1U<<2) | (3U<<23));
    p += 4;

    p = put_xdi(p, 0x3010, 4);      // barometric pressure, Pa, with noise so AP_Baro sees it change
    put_be32_ptr(p, uint32_t(pressure + rand_float() * 3));
    p += 4;

    // the simulated geoid separation is zero: ellipsoid height == AMSL height
    p = put_xdi(p, 0x5042, 12);     // lat/lon
    p = put_fp1632(p, fdm.latitude);
    p = put_fp1632(p, fdm.longitude);

    p = put_xdi(p, 0x5022, 6);      // altitude above ellipsoid
    p = put_fp1632(p, fdm.altitude);

    p = put_xdi(p, 0xD012, 18);     // velocity, NED
    p = put_fp1632(p, fdm.speedN);
    p = put_fp1632(p, fdm.speedE);
    p = put_fp1632(p, fdm.speedD);

    if (with_pvt) {
        // GNSS PVT data, u-blox NAV-PVT layout. Only the fields used by the driver are set
        const auto gps_tow = GPS_Backend::gps_time();
        p = put_xdi(p, 0x7010, 94);
        put_be32_ptr(p, gps_tow.ms);                        // iTOW
        p[20] = 3;                                          // fixType: 3D
        p[21] = 0x01;                                       // flags: gnssFixOK
        p[22] = 12;                                         // numSV
        put_be32_ptr(p+24, int32_t(fdm.longitude * 1e7));
        put_be32_ptr(p+28, int32_t(fdm.latitude * 1e7));
        put_be32_ptr(p+32, int32_t(fdm.altitude * 1000));   // height above ellipsoid, mm
        put_be32_ptr(p+36, int32_t(fdm.altitude * 1000));   // height above MSL, mm
        put_be32_ptr(p+40, 1500);                           // hAcc, mm
        put_be32_ptr(p+44, 2500);                           // vAcc, mm
        put_be32_ptr(p+68, 100);                            // sAcc, mm/s
        put_be16_ptr(p+86, 150);                            // vDOP * 100
        put_be16_ptr(p+88, 100);                            // hDOP * 100
        p += 94;
    }

    send_message(0x36, payload, p - payload);
}

/*
  send MTData2 at 50Hz, with GNSS PVT data at 5Hz. The simulated MTi is
  configured as described on the wiki. The local frame is aligned to NED
  in MT Manager, so the data is NED/FRD while the XDI coordinate bits
  still say ENU, as on a real MTi set up this way
 */
void Xsens::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }
    const uint32_t now_us = AP_HAL::micros();
    if (now_us - last_pkt_us >= 1000000U / rate_hz) {
        last_pkt_us = now_us;
        send_mtdata2(packets_sent % (rate_hz / 5) == 0);
        packets_sent++;
    }
}
