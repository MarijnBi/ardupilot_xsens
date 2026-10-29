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
  simulate a serial Xsens MTi GNSS/INS (e.g. MTi-680G)

  Usage example:
     SERIAL4_PROTOCOL = 36
     AHRS_EKF_TYPE = 11
     EAHRS_TYPE = 12

     sim_vehicle.py -v Plane -D --console --map -A --serial4=sim:Xsens
*/

#pragma once

#include "SIM_SerialDevice.h"

namespace SITL {

class Xsens : public SerialDevice {
public:
    Xsens();

    // update state
    void update(void);

private:
    void send_message(uint8_t msg_id, const uint8_t *payload, uint16_t len);
    void send_mtdata2(bool with_pvt);

    const uint16_t rate_hz = 50;
    uint32_t last_pkt_us;
    uint32_t packets_sent;

};

}
