/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
Multiprotocol is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with Multiprotocol.  If not, see <http://www.gnu.org/licenses/>.
 
 Thanks to  Goebish ,Ported  from his deviation firmware
 */

#if defined(V761_NRF24L01_INO)

#include "iface_nrf24l01.h"

#define V761_PACKET_PERIOD	7060 // Timeout for callback in uSec
#define V761_INITIAL_WAIT	500
#define V761_PACKET_SIZE	8
#define V761_BIND_COUNT	200

//Fx chan management
#define V761_BIND_FREQ	0x28
#define V761_RF_NUM_CHANNELS 3

enum 
   {
    V761_BIND1 = 0,
    V761_BIND2,
    V761_DATA
   };

// Channel 5 - Gyro mode is packet 5
if(CH5_SW)		// Mode Expert Gyro off
	flags = 0x0c;
else
	if(Channel_data[CH5] < CHANNEL_MIN_COMMAND)
		flags = 0x0a;			// Biggner mode -Included for possible variation between bigginer mode and Midd, currently duplicate of midd mode
	else
		flags = 0x0a;			// Midd Mode ( Gyro on no rate limits)

static void __attribute__((unused)) set_checksum()
    {
    uint8_t i;
    uint8_t checksum = packet[0];
    for(i=1; i<V761_PACKET_SIZE-2; i++)
        checksum += packet[i];
    if(phase == BIND1) 
	{
        packet[6] = checksum ^ 0xff;
        packet[7] = packet[6];
    	}
    else 
	{
        checksum += packet[6];
        packet[7] = checksum ^ 0xff;
    	}
    }


static void __attribute__((unused)) send_packet(uint8_t bind)
{
	if(bind)
	{
		packet[0] = txid[0];
        	packet[1] = txid[1];
        	packet[2] = txid[2];
        	packet[3] = txid[3];
        	packet[4] = rf_channels[1];
        	packet[5] = rf_channels[2];
        	if(phase == BIND2) 
            		packet[6] = 0xf0; // ?
	}
	else
	{ 
        packet[0] = convert_channel_16b_limit(THROTTLE,0,0xff); // throttle       
        packet[1] = convert_channel_16b_limit(RUDDER, 0, 0x7f); // rudder
        packet[2] = convert_channel_16b_limit(ELEVATOR, 0, 0x7f); // elevator
        packet[3] = 0x3f; // optional aileron channel for 4ch version?
        packet[5] = (packet_counter++ / 3) * 0x40;
        packet[4] = (packet[5] == 0x40) ? 0x1a : 0x20;
        packet[5] |= flags;
        packet[6] = 0x80; // unknown 
	//packet counter
        if(packet_counter >= 12)
            packet_counter = 0;
        NRF24L01_WriteReg(NRF24L01_05_RF_CH, V761_rf_channels[rf_chan++]);  //unsure of this command execution to implement the channel hop( taken from gobish original)
        if(rf_chan >= V761_RF_NUM_CHANNELS)
            rf_chan = 0;
    	}

	set_checksum()
    	// Power on, TX mode, 2byte CRC
    	XN297_Configure(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO) | BV(NRF24L01_00_PWR_UP));
    	NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70); // flush interrupts
    	NRF24L01_FlushTx();
    	// transmit packet
    	XN297_WritePayload(packet, V761_PACKET_SIZE);
}

static __attribute__((unused)) V761_init(
{
    NRF24L01_Initialize();
    NRF24L01_SetTxRxMode(TX_EN);
    
  
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);		// No Auto Acknowldgement on all data pipes
    NRF24L01_WriteReg(NRF24L01_02_EN_RXADDR, 0x01);  	// Enable data pipe 0 only
    NRF24L01_WriteReg(NRF24L01_03_SETUP_AW, 0x02);   	// set address length (4 bytes)
    NRF24L01_WriteReg(NRF24L01_04_SETUP_RETR, 0x00); 	// no retransmits
    NRF24L01_SetBitrate(NRF24L01_BR_1M);             	// 1Mbps
    NRF24L01_SetPower();
    NRF24L01_Activate(0x73);                         	// Activate feature register
    NRF24L01_WriteReg(NRF24L01_1C_DYNPD, 0x00);      	// Disable dynamic payload length on all pipes
    NRF24L01_WriteReg(NRF24L01_1D_FEATURE, 0x01);
    NRF24L01_Activate(0x73);
  
}


uint16_t V761_callback()
{
	switch(phase)
	{
		case V761_BIND1:
			bind_counter--;
            		packet_counter ++;
           		NRF24L01_WriteReg(NRF24L01_05_RF_CH, V761_BIND_FREQ);
			XN297_SetTXAddr((uint8_t*)"\x34\x43\x10\x10", 4);
            		send_packet();
           		if(packet_counter >= 20) 
				{
                		packet_counter = 0;
                		phase = V761_BIND2;
            			}
            		return 15730;
            		break;

		case GW008_BIND2:
			// switch to RX mode
			bind_counter--;
           		packet_counter ++;
            		NRF24L01_WriteReg(NRF24L01_05_RF_CH, rf_channels[0]);
            		XN297_SetTXAddr(txid, 4);
            		send_packet();
            		if(bind_counter <= 0) 
				{
                		packet_counter = 0;
                		phase = V761_DATA;
                		//PROTOCOL_SetBindState(0);
                		return 15730;
            			}
            		if(packet_counter >= 20) 
				{
                		packet_counter = 0;
                		phase = V761_BIND1;
            			}
            		return 15730;
            		break;

			
		case V761_DATA:
			send_packet(0);
			break;
	}
	return V761_PACKET_PERIOD;
}

static void __attribute__((unused)) V761_initialize_txid()
	{
    	// TODO: try arbitrary txid & frequencies (except rf_channels[0])
	memcpy(txid,(uint8_t *)"\x6f\x2c\xb1\x93",4);
	memcpy(rf_channels,(uint8_t *)"\x14\x1e\x4b",3);
       	//rf_channels[0] = 0x14; // not sure if this one is const or calculated ...
    	//rf_channels[1] = 0x1e;
    	//rf_channels[2] = 0x4b;
	}

uint16_t initV761(void)
	{
	BIND_IN_PROGRESS;	// autobind protocol
    	bind_counter = V761_BIND_COUNT;
	V761_initialize_txid();
	phase = V761_BIND1;
	V761_init();
	rf_chan = 0
	return	V761_INITIAL_WAIT;
	}

#endif
