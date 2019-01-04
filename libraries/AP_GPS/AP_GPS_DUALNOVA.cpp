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

//  DUALNovatel/Tersus/ComNav GPS driver for ArduPilot.
//  Code by lin zihao
//  Derived from http://www.novatel.com/assets/Documents/Manuals/om-20000129.pdf

#include "AP_GPS.h"
#include "AP_GPS_DUALNOVA.h"
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

#define DUALNOVA_DEBUGGING 1

#if DUALNOVA_DEBUGGING
#include <cstdio>
 # define Debug(fmt, args ...)                  \
do {                                            \
    printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
 # define Debug(fmt, args ...)
#endif

AP_GPS_DUALNOVA::AP_GPS_DUALNOVA(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                       AP_HAL::UARTDriver *_port) :
    AP_GPS_NOVA(_gps, _state, _port),
    _new_heading(0)
{
    
    dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;
    /*
    const char *init_str = _initialisation_blob[0];
    const char *init_str1 = _initialisation_blob[1];
    
    port->write((const uint8_t*)init_str, strlen(init_str));
    port->write((const uint8_t*)init_str1, strlen(init_str1));
    */
}

// Process all bytes available from the stream
//
bool
AP_GPS_DUALNOVA::read(void)
{
    uint32_t now = AP_HAL::millis();
	Debug("_init_blob_index= %u\n", _init_blob_index);

    if (_init_blob_index < (sizeof(_initialisation_blob) / sizeof(_initialisation_blob[0]))) {
        const char *init_str = _initialisation_blob[_init_blob_index];

        if (now > _init_blob_time) {
            port->write((const uint8_t*)init_str, strlen(init_str));
            _init_blob_time = now + 200;
            _init_blob_index++;
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        uint8_t temp = port->read();
        ret |= parse(temp);
    }
    
    return ret;
}

bool
AP_GPS_DUALNOVA::parse(uint8_t temp)
{
    switch (dualnova_msg.nova_state)
    {
        default:
        case dualnova_msg_parser::PREAMBLE1:
            if (temp == NOVA_PREAMBLE1)
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE2;
            dualnova_msg.read = 0;
            break;
        case dualnova_msg_parser::PREAMBLE2:
            if (temp == NOVA_PREAMBLE2)
            {
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE3;
            }
            else
            {
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;
            }
            break;
        case dualnova_msg_parser::PREAMBLE3:
            if (temp == NOVA_PREAMBLE3)
            {
                dualnova_msg.nova_state = dualnova_msg_parser::HEADERLENGTH;
            }
            else
            {
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;
            }
            break;
        case dualnova_msg_parser::HEADERLENGTH:
            Debug("DUALNOVA HEADERLENGTH\n");
            dualnova_msg.header.data[0] = NOVA_PREAMBLE1;
            dualnova_msg.header.data[1] = NOVA_PREAMBLE2;
            dualnova_msg.header.data[2] = NOVA_PREAMBLE3;
            dualnova_msg.header.data[3] = temp;
            dualnova_msg.header.nova_headeru.headerlength = temp;
            dualnova_msg.nova_state = dualnova_msg_parser::HEADERDATA;
            dualnova_msg.read = 4;
            break;
        case dualnova_msg_parser::HEADERDATA:
            if (dualnova_msg.read >= sizeof(dualnova_msg.header.data)) {
                Debug("parse header overflow length=%u\n", (unsigned)dualnova_msg.read);
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;
                break;
            }
            dualnova_msg.header.data[dualnova_msg.read] = temp;
            dualnova_msg.read++;
            if (dualnova_msg.read >= dualnova_msg.header.nova_headeru.headerlength)
            {
                dualnova_msg.nova_state = dualnova_msg_parser::DATA;
            }
            break;
        case dualnova_msg_parser::DATA:
            if (dualnova_msg.read >= sizeof(dualnova_msg.data)) {
                Debug("parse data overflow length=%u msglength=%u\n", (unsigned)dualnova_msg.read,dualnova_msg.header.nova_headeru.messagelength);
                dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;
                break;
            }
            dualnova_msg.data.bytes[dualnova_msg.read - dualnova_msg.header.nova_headeru.headerlength] = temp;
            dualnova_msg.read++;
            if (dualnova_msg.read >= (dualnova_msg.header.nova_headeru.messagelength + dualnova_msg.header.nova_headeru.headerlength))
            {
                Debug("DUALNOVA DATA exit\n");
                dualnova_msg.nova_state = dualnova_msg_parser::CRC1;
            }
            break;
        case dualnova_msg_parser::CRC1:
            dualnova_msg.crc = (uint32_t) (temp << 0);
            dualnova_msg.nova_state = dualnova_msg_parser::CRC2;
            break;
        case dualnova_msg_parser::CRC2:
            dualnova_msg.crc += (uint32_t) (temp << 8);
            dualnova_msg.nova_state = dualnova_msg_parser::CRC3;
            break;
        case dualnova_msg_parser::CRC3:
            dualnova_msg.crc += (uint32_t) (temp << 16);
            dualnova_msg.nova_state = dualnova_msg_parser::CRC4;
            break;
        case dualnova_msg_parser::CRC4:
            dualnova_msg.crc += (uint32_t) (temp << 24);
            dualnova_msg.nova_state = dualnova_msg_parser::PREAMBLE1;

            uint32_t crc = CalculateBlockCRC32((uint32_t)dualnova_msg.header.nova_headeru.headerlength, (uint8_t *)&dualnova_msg.header.data, (uint32_t)0);
            crc = CalculateBlockCRC32((uint32_t)dualnova_msg.header.nova_headeru.messagelength, (uint8_t *)&dualnova_msg.data, crc);

            if (dualnova_msg.crc == crc)
            {
                return process_message();
            }
            else
            {
                Debug("crc failed");
                crc_error_counter++;
            }
            break;
    }

    return false;
}

bool
AP_GPS_DUALNOVA::process_message(void)
{
    uint16_t messageid = dualnova_msg.header.nova_headeru.messageid;

    Debug("DUALNOVA process_message messid=%u\n",messageid);
    
    if (messageid == 42) // bestpos
    {
        const bestpos &bestposu = dualnova_msg.data.bestposu;
        //class AP_GPS_Backend protected:AP_GPS::GPS_State &state; 
        state.time_week = dualnova_msg.header.nova_headeru.week;//< GPS week number
        state.time_week_ms = (uint32_t) dualnova_msg.header.nova_headeru.tow;//< GPS time (milliseconds from start of GPS week)
        state.last_gps_time_ms = AP_HAL::millis();//< the system time we got the last GPS timestamp, milliseconds
        //Location defined in AP_Common.h
        state.location.lat = (int32_t) (bestposu.lat * (double)1e7);//< last fix location
        state.location.lng = (int32_t) (bestposu.lng * (double)1e7);
        state.location.alt = (int32_t) (bestposu.hgt * 100);
		Debug("DUALNOVA lat=%d\n", (int32_t)state.location.lat );
		Debug("DUALNOVA lng=%d\n", (int32_t)state.location.lng );
		Debug("DUALNOVA alt=%d\n", (int32_t)state.location.alt );

        state.num_sats = bestposu.svsused;//< Number of visible satellites

        state.horizontal_accuracy = (float) ((bestposu.latsdev + bestposu.lngsdev)/2);//< 3D velocity accuracy estimate in m/s
        state.vertical_accuracy = (float) bestposu.hgtsdev;//< horizontal accuracy estimate in m
        state.have_horizontal_accuracy = true;
        state.have_vertical_accuracy = true;

        if (bestposu.solstat == 0) // have a solution
        {
            switch (bestposu.postype)
            {
                case 16:
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    break;
                case 17: // psrdiff
                case 18: // waas
                case 20: // omnistar
                case 68: // ppp_converg
                case 69: // ppp
                    state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                    break;
                case 32: // l1 float
                case 33: // iono float
                case 34: // narrow float
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                    break;
                case 48: // l1 int
                case 50: // narrow int
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                    break;
                case 0: // NONE
                case 1: // FIXEDPOS
                case 2: // FIXEDHEIGHT
                default:
                    state.status = AP_GPS::NO_FIX;
                    break;
            }
        }
        else
        {
            state.status = AP_GPS::NO_FIX;
        }
        
        _new_position = true;
    }

    if (messageid == 99) // bestvel
    {
        const bestvel &bestvelu = dualnova_msg.data.bestvelu;

        state.ground_speed = (float) bestvelu.horspd;//< ground speed in m/sec
        state.ground_course = (float) bestvelu.trkgnd;//< ground course in degrees
        fill_3d_velocity();//< 3D velocity in m/s, in NED format
        state.velocity.z = -(float) bestvelu.vertspd;//< 3D velocity in m/s, in NED format
        state.have_vertical_velocity = true;
        
        _last_vel_time = (uint32_t) dualnova_msg.header.nova_headeru.tow;
        _new_speed = true;
    }

    if (messageid == 174) // psrdop
    {
        const psrdop &psrdopu = dualnova_msg.data.psrdopu;

        state.hdop = (uint16_t) (psrdopu.hdop*100);//< horizontal dilution of precision in cm
        state.vdop = (uint16_t) (psrdopu.htdop*100);//< vertical dilution of precision in cm
        return false;
    }

    if (messageid == 971) //heading
    {
        const bestheading &bestheadu = dualnova_msg.data.bestheadu;

        state.heading = bestheadu.heading;//< heading in degrees
        state.pitch = bestheadu.pitch;//< pitch in degrees
        state.heading_accuracy = bestheadu.hdgsdev;
        state.pitch_accuracy = bestheadu.pthsdev;
        state.have_heading_accuracy = true;
        state.have_pitch_accuracy = true;
        Debug("DUALNOVA heading=%f\n", (double)state.heading);
        Debug("DUALNOVA pitch=%f\n", (double)state.pitch);
        Debug("DUALNOVA heading_accuracy=%f\n", (double)state.heading_accuracy);
        Debug("DUALNOVA pitch_accuracy=%f\n", (double)state.pitch_accuracy);

        _last_heading_time = (uint32_t)dualnova_msg.header.nova_headeru.tow;
        _new_heading = true;
/**/

    }

    // ensure out position and velocity stay insync
    //if (_new_position && _new_speed && _new_heading && _last_heading_time == state.time_week_ms && _last_vel_time == state.time_week_ms) {
    //    _new_speed = _new_position = _new_heading = false;
    if (_new_position && _new_speed && _last_vel_time == state.time_week_ms) {
        _new_speed = _new_position = false;
        
        return true;
    }
    
    return false;
}
/*
void
AP_GPS_NOVA::inject_data(const uint8_t *data, uint16_t len)
{
    if (port->txspace() > len) {
        last_injected_data_ms = AP_HAL::millis();
        port->write(data, len);
    } else {
        Debug("NOVA: Not enough TXSPACE");
    }
}

#define CRC32_POLYNOMIAL 0xEDB88320L
uint32_t AP_GPS_NOVA::CRC32Value(uint32_t icrc)
{
    int i;
    uint32_t crc = icrc;
    for ( i = 8 ; i > 0; i-- )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            crc >>= 1;
    }
    return crc;
}

uint32_t AP_GPS_NOVA::CalculateBlockCRC32(uint32_t length, uint8_t *buffer, uint32_t crc)
{
    while ( length-- != 0 )
    {
        crc = ((crc >> 8) & 0x00FFFFFFL) ^ (CRC32Value(((uint32_t) crc ^ *buffer++) & 0xff));
    }
    return( crc );
}
*/
