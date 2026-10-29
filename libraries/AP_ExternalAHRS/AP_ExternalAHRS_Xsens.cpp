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

  The driver does not configure the MTi. The MTData2 output (items and
  rates) is set with MT Manager, see the ArduPilot wiki. The MTi must be
  set up in MT Manager to output in ArduPilot's frames: NED velocity and
  orientation, with a mounting alignment that makes the sensor frame the
  vehicle frame (x forward, y right, z down). The data is used as received;
  the pre-arm check reports items that are missing from the output.
 */

#include "AP_ExternalAHRS_Xsens.h"

#if AP_EXTERNAL_AHRS_XSENS_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

// Xbus framing: preamble, bus ID, message ID, length (0xFF: 16 bit length follows), payload, checksum
static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
static constexpr uint8_t XBUS_MASTERDEVICE = 0xFF;
static constexpr uint8_t XBUS_EXTENDED_LENGTH = 0xFF;
static constexpr uint8_t XBUS_MTDATA2 = 0x36;

// MTData2 data identifiers (XDI) without the format bits. The low nibble of an XDI
// gives the format: bits 0-1 precision, bits 2-3 coordinate system. The coordinate
// bits are ignored: a NED local frame alignment gives NED data with ENU bits
enum XDI : uint16_t {
    TEMPERATURE         = 0x0810,
    UTC_TIME            = 0x1010,
    QUATERNION          = 0x2010,
    BARO_PRESSURE       = 0x3010,
    ACCELERATION        = 0x4020,
    ALTITUDE_ELLIPSOID  = 0x5020,
    LAT_LON             = 0x5040,
    GNSS_PVT_DATA       = 0x7010,
    RATE_OF_TURN        = 0x8020,
    MAGNETIC_FIELD      = 0xC020,
    VELOCITY            = 0xD010,
    STATUS_WORD         = 0xE020,
};
static constexpr uint16_t XDI_TYPE_MASK = 0xFFF0;
static constexpr uint16_t XDI_PRECISION_MASK = 0x0003;

// Sample::have bits
enum {
    HAVE_STATUS = 1U<<0, HAVE_TEMP = 1U<<1, HAVE_QUAT = 1U<<2, HAVE_ACCEL = 1U<<3,
    HAVE_GYRO = 1U<<4, HAVE_MAG = 1U<<5, HAVE_BARO = 1U<<6, HAVE_LATLON = 1U<<7,
    HAVE_ALT = 1U<<8, HAVE_VEL = 1U<<9, HAVE_UTC = 1U<<10, HAVE_PVT = 1U<<11,
};

// items that must be in the MTData2 output, with their MT Manager names.
// Sensor items are only required when used, see EAHRS_SENSORS.
// Items the MTi always outputs must be recent, so a changed output configuration
// is noticed. The MTi only outputs the GNSS dependent items with a GNSS position,
// so for those it is enough that they were received once; their freshness is
// covered by AP_GPS and get_filter_status()
static const struct {
    uint16_t have;
    uint16_t sensor;    // AP_ExternalAHRS::AvailableSensor, 0 if always required
    bool gnss;          // only output by the MTi with a GNSS position
    const char *name;
} required_items[] {
    { HAVE_QUAT,    0, false, "Quaternion" },
    { HAVE_STATUS,  0, false, "StatusWord" },
    { HAVE_UTC,     0, true,  "UtcTime" },
    { HAVE_LATLON,  0, true,  "LatLon" },
    { HAVE_ALT,     0, true,  "AltitudeEllipsoid" },
    { HAVE_VEL,     0, true,  "VelocityXYZ" },
    { HAVE_PVT,     0, true,  "GnssPvtData" },
    { HAVE_ACCEL,   uint16_t(AP_ExternalAHRS::AvailableSensor::IMU),     false, "Acceleration" },
    { HAVE_GYRO,    uint16_t(AP_ExternalAHRS::AvailableSensor::IMU),     false, "RateOfTurn" },
    { HAVE_MAG,     uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS), false, "MagneticField" },
    { HAVE_BARO,    uint16_t(AP_ExternalAHRS::AvailableSensor::BARO),    false, "BaroPressure" },
};

// StatusWord bits
static constexpr uint32_t STATUS_FILTER_VALID = 1U << 1;
static constexpr uint32_t STATUS_FILTER_MODE_MASK = 3U << 23;
static constexpr uint32_t STATUS_FILTER_MODE_WITH_GNSS = 3U << 23;

static constexpr uint32_t GNSS_TIMEOUT_MS = 5000;   // GNSS PVT data older than this is stale
static constexpr uint32_t ATTITUDE_TIMEOUT_MS = 500; // healthy() needs a quaternion this recent
static constexpr uint32_t ITEM_TIMEOUT_MS = 3000;   // pre-arm: an output item older than this is missing
static constexpr uint32_t GPS_POST_PERIOD_MS = 100; // 10Hz to AP_GPS

AP_ExternalAHRS_Xsens::AP_ExternalAHRS_Xsens(AP_ExternalAHRS *_frontend,
                                             AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (uart == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "Xsens: no UART");
        return;
    }

    // The IMU is not used by default: at the MTData2 output rate it is too slow for the
    // rate controllers, which use the autopilot's own IMUs. Set EAHRS_SENSORS bit 1 to use it.
    set_default_sensors(uint16_t(AP_ExternalAHRS::AvailableSensor::GPS) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::BARO) |
                        uint16_t(AP_ExternalAHRS::AvailableSensor::COMPASS));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_Xsens::update_thread, void),
                                      "Xsens", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_BoardConfig::allocation_error("Xsens thread");
    }
}

int8_t AP_ExternalAHRS_Xsens::get_port(void) const
{
    return uart ? port_num : -1;
}

void AP_ExternalAHRS_Xsens::update_thread()
{
    uart->begin(baudrate);

    while (true) {
        hal.scheduler->delay_microseconds(1000);
        check_uart();
    }
}

/*
  read bytes from the UART and handle all complete MTData2 messages
 */
void AP_ExternalAHRS_Xsens::check_uart()
{
    const ssize_t n = uart->read(&buf[buf_len], sizeof(buf) - buf_len);
    if (n <= 0) {
        return;
    }
    buf_len += n;

    uint16_t ofs = 0;
    while (ofs < buf_len) {
        if (buf[ofs] != XBUS_PREAMBLE) {
            ofs++;
            continue;
        }
        const uint16_t avail = buf_len - ofs;
        if (avail < 4) {
            break;  // wait for the rest of the header
        }
        if (buf[ofs+1] != XBUS_MASTERDEVICE) {
            ofs++;
            continue;
        }
        uint16_t header_len = 4;
        uint16_t payload_len = buf[ofs+3];
        if (payload_len == XBUS_EXTENDED_LENGTH) {
            if (avail < 6) {
                break;
            }
            header_len = 6;
            payload_len = be16toh_ptr(&buf[ofs+4]);
        }
        // 32 bit: an extended length close to 0xFFFF must not wrap around
        const uint32_t msg_len = uint32_t(header_len) + payload_len + 1;
        if (msg_len > sizeof(buf)) {
            ofs++;  // can't be a valid message
            continue;
        }
        if (avail < msg_len) {
            break;  // wait for the rest of the message
        }
        // checksum: sum of all bytes after the preamble is 0
        uint8_t sum = 0;
        for (uint16_t i = 1; i < msg_len; i++) {
            sum += buf[ofs+i];
        }
        if (sum != 0) {
            ofs++;  // resync from the next byte
            continue;
        }
        if (buf[ofs+2] == XBUS_MTDATA2) {
            Sample s {};
            bool got_pvt = false;
            if (parse_mtdata2(&buf[ofs+header_len], payload_len, s, pvt, got_pvt)) {
                if (got_pvt) {
                    have_pvt = true;
                    last_pvt_ms = AP_HAL::millis();
                    // the MTi outputs the altitude above the ellipsoid, ArduPilot uses AMSL
                    if (pvt.fix_type >= 2 && pvt.fix_type <= 4) {
                        geoid_separation_m = (pvt.height_mm - pvt.height_msl_mm) * 0.001f;
                        have_geoid_separation = true;
                    }
                }
                handle_sample(s);
            }
        }
        ofs += msg_len;
    }

    // keep the unprocessed bytes
    buf_len -= ofs;
    memmove(buf, &buf[ofs], buf_len);
}

/*
  read count values in the precision given by the XDI format bits:
  0: float32, 1: fp12.20, 2: fp16.32, 3: float64.
  Returns false if size does not match
 */
static bool read_values(const uint8_t *p, uint16_t xdi, uint8_t size, double *v, uint8_t count)
{
    static const uint8_t value_size[] { 4, 4, 6, 8 };
    const uint8_t precision = xdi & XDI_PRECISION_MASK;
    if (size != count * value_size[precision]) {
        return false;
    }
    for (uint8_t i = 0; i < count; i++, p += value_size[precision]) {
        switch (precision) {
        case 0:
            v[i] = be32tofloat_ptr(p);
            break;
        case 1:
            v[i] = int32_t(be32toh_ptr(p)) * (1.0 / 1048576.0);
            break;
        case 2: {
            // 32 bit fraction followed by 16 bit signed integer
            const int64_t fp = int64_t(int16_t(be16toh_ptr(p+4))) * 4294967296LL + int64_t(be32toh_ptr(p));
            v[i] = double(fp) * (1.0 / 4294967296.0);  // keep double precision with -fsingle-precision-constant
            break;
        }
        case 3: {
            const uint64_t u = (uint64_t(be32toh_ptr(p)) << 32) | be32toh_ptr(p+4);
            memcpy(&v[i], &u, 8);
            break;
        }
        }
    }
    return true;
}

/*
  decode a MTData2 payload. Items with an unexpected size are skipped
 */
bool AP_ExternalAHRS_Xsens::parse_mtdata2(const uint8_t *payload, uint16_t len, Sample &s, GnssPvt &gnss, bool &got_pvt) const
{
    uint16_t ofs = 0;
    while (ofs + 3 <= len) {
        const uint16_t xdi = be16toh_ptr(&payload[ofs]);
        const uint8_t size = payload[ofs+2];
        const uint8_t *p = &payload[ofs+3];
        ofs += 3 + size;
        if (ofs > len) {
            return false;
        }
        double v[4];
        switch (xdi & XDI_TYPE_MASK) {
        case XDI::STATUS_WORD:
            if (size == 4) {
                s.status_word = be32toh_ptr(p);
                s.have |= HAVE_STATUS;
            }
            break;
        case XDI::TEMPERATURE:
            if (read_values(p, xdi, size, v, 1)) {
                s.temperature = v[0];
                s.have |= HAVE_TEMP;
            }
            break;
        case XDI::BARO_PRESSURE:
            if (size == 4) {
                s.pressure_pa = be32toh_ptr(p);
                s.have |= HAVE_BARO;
            }
            break;
        case XDI::LAT_LON:
            if (read_values(p, xdi, size, v, 2)) {
                s.lat = v[0];
                s.lon = v[1];
                s.have |= HAVE_LATLON;
            }
            break;
        case XDI::ALTITUDE_ELLIPSOID:
            if (read_values(p, xdi, size, v, 1)) {
                s.alt_ellipsoid = v[0];
                s.have |= HAVE_ALT;
            }
            break;
        case XDI::QUATERNION:
            if (read_values(p, xdi, size, v, 4)) {
                s.quat = Quaternion(v[0], v[1], v[2], v[3]);
                s.have |= HAVE_QUAT;
            }
            break;
        case XDI::ACCELERATION:
            if (read_values(p, xdi, size, v, 3)) {
                s.accel = Vector3f(v[0], v[1], v[2]);
                s.have |= HAVE_ACCEL;
            }
            break;
        case XDI::RATE_OF_TURN:
            if (read_values(p, xdi, size, v, 3)) {
                s.gyro = Vector3f(v[0], v[1], v[2]);
                s.have |= HAVE_GYRO;
            }
            break;
        case XDI::MAGNETIC_FIELD:
            if (read_values(p, xdi, size, v, 3)) {
                s.mag = Vector3f(v[0], v[1], v[2]);
                s.have |= HAVE_MAG;
            }
            break;
        case XDI::VELOCITY:
            if (read_values(p, xdi, size, v, 3)) {
                s.vel_ned = Vector3f(v[0], v[1], v[2]);
                s.have |= HAVE_VEL;
            }
            break;
        case XDI::UTC_TIME:
            if (size == 12) {
                s.utc_ns = be32toh_ptr(p);
                s.utc = {};
                s.utc.tm_year = be16toh_ptr(p+4) - 1900;
                s.utc.tm_mon = p[6] - 1;
                s.utc.tm_mday = p[7];
                s.utc.tm_hour = p[8];
                s.utc.tm_min = p[9];
                s.utc.tm_sec = p[10];
                s.have |= HAVE_UTC;
            }
            break;
        case XDI::GNSS_PVT_DATA:
            // field offsets as in the u-blox NAV-PVT message
            if (size == 94) {
                gnss.fix_type = p[20];
                gnss.flags = p[21];
                gnss.num_sv = p[22];
                gnss.height_mm = int32_t(be32toh_ptr(p+32));
                gnss.height_msl_mm = int32_t(be32toh_ptr(p+36));
                gnss.h_acc_mm = be32toh_ptr(p+40);
                gnss.v_acc_mm = be32toh_ptr(p+44);
                gnss.s_acc_mms = be32toh_ptr(p+68);
                gnss.vdop = be16toh_ptr(p+86);
                gnss.hdop = be16toh_ptr(p+88);
                s.have |= HAVE_PVT;
                got_pvt = true;
            }
            break;
        }
    }
    return true;
}

void AP_ExternalAHRS_Xsens::handle_sample(const Sample &s)
{
    const uint32_t now_ms = AP_HAL::millis();
    last_pkt_ms = now_ms;
    for (uint8_t i = 0; i < NUM_ITEMS; i++) {
        if (s.have & (1U << i)) {
            item_ms[i] = now_ms;
        }
    }
    if (s.have & HAVE_STATUS) {
        status_word = s.status_word;
        have_status_word = true;
    }

    // position is only used once the geoid separation is known from a GNSS fix
    const bool have_position = (s.have & HAVE_LATLON) && (s.have & HAVE_ALT) && have_geoid_separation;
    Location loc;
    if (have_position) {
        loc = Location{int32_t(s.lat * 1e7), int32_t(s.lon * 1e7), int32_t((s.alt_ellipsoid - geoid_separation_m) * 100), Location::AltFrame::ABSOLUTE};
        if (s.have & HAVE_VEL) {
            last_pos_ms = now_ms;
        }
    }

    {
        WITH_SEMAPHORE(state.sem);
        if (s.have & HAVE_ACCEL) {
            state.accel = s.accel;
        }
        if (s.have & HAVE_GYRO) {
            state.gyro = s.gyro;
        }
        if (s.have & HAVE_QUAT) {
            state.quat = s.quat;
            state.have_quaternion = true;
        }
        if (s.have & HAVE_VEL) {
            state.velocity = s.vel_ned;
            state.have_velocity = true;
        }
        if (have_position) {
            state.location = loc;
            state.have_location = true;
            state.last_location_update_us = AP_HAL::micros();
            if (!state.have_origin && filter_gnss_aided()) {
                state.origin = loc;
                state.have_origin = true;
            }
        }
    }

    const float temperature = (s.have & HAVE_TEMP) ? s.temperature : 25;

    if ((s.have & HAVE_ACCEL) && (s.have & HAVE_GYRO)) {
        AP_ExternalAHRS::ins_data_message_t ins {
            accel: s.accel,
            gyro: s.gyro,
            temperature: temperature,
        };
        AP::ins().handle_external(ins);
    }

#if AP_COMPASS_EXTERNALAHRS_ENABLED
    if (s.have & HAVE_MAG) {
        // the MTi outputs normalised arbitrary units, 1 a.u. is approximately 490 mGauss
        AP_ExternalAHRS::mag_data_message_t mag {
            field: s.mag * 490.0f,
        };
        AP::compass().handle_external(mag);
    }
#endif

#if AP_BARO_EXTERNALAHRS_ENABLED
    if (s.have & HAVE_BARO) {
        AP_ExternalAHRS::baro_data_message_t baro {
            instance: 0,
            pressure_pa: s.pressure_pa,
            temperature: temperature,
        };
        AP::baro().handle_external(baro);
    }
#endif

    if (have_position && (s.have & HAVE_VEL) && (s.have & HAVE_UTC)) {
        post_gps(s, loc);
    }
}

/*
  send the fused position and velocity with the GNSS fix status to AP_GPS.
  This is rate limited to 10Hz with a fixed phase: AP_GPS assumes a constant
  rate of at most 20Hz when it derives the message timing from ms_tow.
 */
void AP_ExternalAHRS_Xsens::post_gps(const Sample &s, const Location &loc)
{
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_gps_post_ms < GPS_POST_PERIOD_MS) {
        return;
    }
    last_gps_post_ms += GPS_POST_PERIOD_MS;
    if (now_ms - last_gps_post_ms >= GPS_POST_PERIOD_MS) {
        // first message, or after a gap
        last_gps_post_ms = now_ms;
    }

    // GPS time from the UTC time of this sample. AP_GPS wants the full week number
    const uint64_t gps_ms = uint64_t(ap_mktime(&s.utc)) * 1000ULL + s.utc_ns / 1000000U - UNIX_OFFSET_MSEC;

    const bool gnss_ok = gnss_valid(now_ms);

    AP_ExternalAHRS::gps_data_message_t gps {
        gps_week: uint16_t(gps_ms / AP_MSEC_PER_WEEK),
        ms_tow: uint32_t(gps_ms % AP_MSEC_PER_WEEK),
        fix_type: gnss_ok ? fix_type() : AP_GPS_FixType::NONE,
        satellites_in_view: gnss_ok ? pvt.num_sv : uint8_t(0),
        horizontal_pos_accuracy: pvt.h_acc_mm * 0.001f,
        vertical_pos_accuracy: pvt.v_acc_mm * 0.001f,
        horizontal_vel_accuracy: pvt.s_acc_mms * 0.001f,
        hdop: gnss_ok ? pvt.hdop : float(GPS_UNKNOWN_DOP),
        vdop: gnss_ok ? pvt.vdop : float(GPS_UNKNOWN_DOP),
        longitude: loc.lng,
        latitude: loc.lat,
        msl_altitude: loc.alt,
        ned_vel_north: s.vel_ned.x,
        ned_vel_east: s.vel_ned.y,
        ned_vel_down: s.vel_ned.z,
    };

    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
}

// convert the u-blox style fix type and flags in the GNSS PVT data
AP_GPS_FixType AP_ExternalAHRS_Xsens::fix_type() const
{
    switch (pvt.fix_type) {
    case 2:
        return AP_GPS_FixType::FIX_2D;
    case 3:
        // carrSoln must be checked before diffSoln, RTK solutions also have diffSoln set
        switch ((pvt.flags >> 6) & 0x03) {
        case 1:
            return AP_GPS_FixType::RTK_FLOAT;
        case 2:
            return AP_GPS_FixType::RTK_FIXED;
        }
        return (pvt.flags & 0x02) ? AP_GPS_FixType::DGPS : AP_GPS_FixType::FIX_3D;
    case 4:
        return AP_GPS_FixType::FIX_3D;  // GNSS + dead reckoning
    default:
        return AP_GPS_FixType::NONE;
    }
}

bool AP_ExternalAHRS_Xsens::gnss_valid(uint32_t now_ms) const
{
    const uint32_t pvt_ms = last_pvt_ms;
    return have_pvt && now_ms - pvt_ms < GNSS_TIMEOUT_MS;
}

// true when the MTi reports a valid orientation filter
bool AP_ExternalAHRS_Xsens::filter_valid() const
{
    return have_status_word && (status_word & STATUS_FILTER_VALID) != 0;
}

// true when the MTi filter is in "with GNSS" mode, i.e. its position is GNSS aided
bool AP_ExternalAHRS_Xsens::filter_gnss_aided() const
{
    return have_status_word && (status_word & STATUS_FILTER_MODE_MASK) == STATUS_FILTER_MODE_WITH_GNSS;
}

// true if the item (a HAVE_* bit) was received within timeout_ms
bool AP_ExternalAHRS_Xsens::item_recent(uint16_t have, uint32_t now_ms, uint32_t timeout_ms) const
{
    for (uint8_t i = 0; i < NUM_ITEMS; i++) {
        if (have == (1U << i)) {
            // the timestamp is written by the Xsens thread, so it can be newer than now_ms
            const uint32_t t_ms = item_ms[i];
            if (t_ms == 0) {
                return false;
            }
            return int32_t(now_ms - t_ms) <= 0 || now_ms - t_ms < timeout_ms;
        }
    }
    return false;
}

/*
  healthy means the MTi is delivering attitude. GNSS quality is reported through
  AP_GPS and get_filter_status(), so losing GNSS does not make it unhealthy
 */
bool AP_ExternalAHRS_Xsens::healthy(void) const
{
    return item_recent(HAVE_QUAT, AP_HAL::millis(), ATTITUDE_TIMEOUT_MS);
}

bool AP_ExternalAHRS_Xsens::initialised(void) const
{
    return last_pkt_ms != 0;
}

bool AP_ExternalAHRS_Xsens::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    // read the timestamp before millis(): it is written by the Xsens thread
    const uint32_t pkt_ms = last_pkt_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (pkt_ms == 0 || now_ms - pkt_ms > ATTITUDE_TIMEOUT_MS) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens unhealthy");
        return false;
    }
    // check the MTData2 output configuration made with MT Manager
    for (const auto &item : required_items) {
        const uint32_t timeout_ms = item.gnss ? UINT32_MAX : ITEM_TIMEOUT_MS;
        if (!item_recent(item.have, now_ms, timeout_ms) &&
            (item.sensor == 0 || AP::externalAHRS().has_sensor(AP_ExternalAHRS::AvailableSensor(item.sensor)))) {
            hal.util->snprintf(failure_msg, failure_msg_len, "Xsens: no %s output", item.name);
            return false;
        }
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens unhealthy");
        return false;
    }
    if (!filter_valid()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "Xsens filter not valid");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_Xsens::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    if (!healthy()) {
        return;
    }
    status.flags.initalized = true;
    status.flags.attitude = filter_valid();
    status.flags.vert_vel = true;
    status.flags.vert_pos = true;

    // horizontal position and velocity are only valid while the MTi outputs a
    // GNSS aided position with a 3D fix; ArduPilot must not use dead reckoning
    const uint32_t pos_ms = last_pos_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (gnss_valid(now_ms) && fix_type() >= AP_GPS_FixType::FIX_3D &&
        pos_ms != 0 && now_ms - pos_ms < 500 && filter_gnss_aided()) {
        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

bool AP_ExternalAHRS_Xsens::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    if (!have_pvt) {
        return false;
    }
    velVar = pvt.s_acc_mms * 0.001f * vel_gate_scale;
    posVar = pvt.h_acc_mm * 0.001f * pos_gate_scale;
    hgtVar = pvt.v_acc_mm * 0.001f * hgt_gate_scale;
    magVar.zero();
    tasVar = 0;
    return true;
}

#endif  // AP_EXTERNAL_AHRS_XSENS_ENABLED
