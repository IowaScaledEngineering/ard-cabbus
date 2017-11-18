/*************************************************************************
Title:    ARD-CABBUS Arduino-based decoder for the NCE Cab Bus
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <util/atomic.h>

#define BAUD 9600
#define QUEUE_DEPTH 64

#define PKT_BUFFER_SIZE  16

uint16_t rxBuffer[QUEUE_DEPTH];
uint8_t headIdx, tailIdx, rxBufferFull;

uint16_t packetBuffer[PKT_BUFFER_SIZE];

void serialInit(void)
{
#include <util/setbaud.h>
  
  UBRR1 = UBRR_VALUE;
  UCSR1A = (USE_2X)?_BV(U2X1):0;
  UCSR1B = _BV(UCSZ12);
  UCSR1C = _BV(UCSZ11) | _BV(UCSZ10);

  /* Enable USART receiver and transmitter and receive complete interrupt */
  UCSR1B |= (_BV(RXCIE1) | _BV(RXEN1) | _BV(TXEN1));
}

ISR(USART1_RX_vect)
{
  uint16_t data = 0;

/*  if(UCSR1B & _BV(RXB81))*/
/*    data |= 0x0100;  // bit 9 set*/

  data |= UDR1;
  
  rxBufferPush(data);
}

void rxBufferInitialize(void)
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    headIdx = tailIdx = 0;
    rxBufferFull = 0;
  }
}

uint8_t rxBufferDepth(void)
{
  uint8_t result;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    if(rxBufferFull)
      return(QUEUE_DEPTH);
    result = ((uint8_t)(headIdx - tailIdx) % QUEUE_DEPTH);
  }
  return(result);
}

uint8_t rxBufferPush(uint16_t data)
{
    // If full, bail with a false
    if (rxBufferFull)
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    rxBuffer[headIdx] = data;
  
    if( ++headIdx >= QUEUE_DEPTH )
      headIdx = 0;
    if (headIdx == tailIdx)
      rxBufferFull = 1;
  }
  return(1);
}

uint16_t rxBufferPop(uint8_t snoop)
{
  uint16_t data;
    if (0 == rxBufferDepth())
      return(0);
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
  {
    data = rxBuffer[tailIdx];
    if(!snoop)
    {
      if( ++tailIdx >= QUEUE_DEPTH )
        tailIdx = 0;
      rxBufferFull = 0;
    }
  }
  return(data);
}

uint8_t adjustCabBusASCII(uint8_t chr)
{
	if(chr & 0x20)
		return(chr & 0x3F);  // Clear bit 6 & 7
	else
		return(chr & 0x7F);  // Clear only bit 7
}


void setup()
{
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

	rxBufferInitialize();

	Serial.begin(57600);
	serialInit();
}

#define CABBUS_STATUS_BROADCAST_CMD    0x01
#define CABBUS_STATUS_DUMB_CAB         0x02
#define CABBUS_STATUS_SMART_CAB        0x04
#define CABBUS_STATUS_TX_PENDING       0x80

uint8_t cabBusStatus = 0;

void loop()
{
	uint16_t data, cmd;
	static uint8_t byte_count = 0;
	uint8_t i, speed;
	uint16_t locomotive;

	if(rxBufferDepth() > 1)
	{
		data = rxBufferPop(0);

		// Dumb Cab:
		// byte 0: --> ping
		// byte 1: <-- response byte 1
		// byte 2: <-- response byte 2
		// byte 3: --> Command byte 1 (11xx xxxx)
		// byte 4: --> Command byte 2 (sometimes doesn't follow the 11xx xxxx rule)
		// byte n: --> More command bytes
		// Might missing a ping following a 2 byte command (byte = 5), but we'll catch it next time we're pinged (FIXME: maybe?)
		//
		// Smart Cab:
		// byte 0: --> ping
		// byte 1: <-- address 1
		// byte 2: <-- address 2
		// byte 3: <-- Command
		// byte 4: <-- Value
		// byte 5: <-- Checksum
		// Note: Bytes 3-5 are allowed to have the MSB set

		if((data & 0xC0) == 0x80)
		{
			// The byte might be a ping, but we need to check the exceptions
			if(
				!( (cabBusStatus & CABBUS_STATUS_DUMB_CAB) && ((4 == byte_count)) ) &&
				!( (cabBusStatus & CABBUS_STATUS_SMART_CAB) && ((3 == byte_count) || (4 == byte_count) || (5 == byte_count)) )
			)
			{
				// Must be a ping, so handle it
				uint8_t address = data & 0x3F;
				
				// Process previous packet, if one exists
				if(byte_count > 1)
				{
					Serial.print("T");
					uint8_t cab = packetBuffer[0] & 0x3F;
					if(cab)
						Serial.print(cab);
					else
						Serial.print('*');
					Serial.print(":");

					if(cabBusStatus & CABBUS_STATUS_SMART_CAB)
					{
						// Smart Cab Response
						cmd = (packetBuffer[1] << 7) | packetBuffer[2];
						if((0 <= cmd) && (cmd <= 0x270F) || (0x2780 <= cmd) && (cmd <= 0x27FF))
						{
							// Locomototive Address
							Serial.print("[ ");
							if(cmd <= 0x270F)
							{
								Serial.print(cmd);
							}
							else
							{
								Serial.print("s");
								Serial.print(cmd - 0x2780);
							}
							Serial.print("] ");
							switch(packetBuffer[3])
							{
								case 0x01:
									break;
								case 0x02:
									break;
								case 0x03:
									// Reverse 128 speed command
									Serial.print("Rev: ");
									Serial.print(packetBuffer[4]);
									Serial.print("/128");
									break;
								case 0x04:
									// Forward 128 speed command
									Serial.print("Fwd: ");
									Serial.print(packetBuffer[4]);
									Serial.print("/128");
									break;
								case 0x05:
									// Reverse E-stop
									Serial.print("ESTOP (Rev)");
									break;
								case 0x06:
									// Forward E-stop
									Serial.print("ESTOP (Fwd)");
									break;
								case 0x07:
									// Function Group 1
									Serial.print("FG1: ");
									if(packetBuffer[4] & 0x10)
										Serial.print("( 0) ");
									else
										Serial.print("(  ) ");
									if(packetBuffer[4] & 0x01)
										Serial.print("( 1) ");
									else
										Serial.print("(  ) ");
									if(packetBuffer[4] & 0x02)
										Serial.print("( 2) ");
									else
										Serial.print("(  ) ");
									if(packetBuffer[4] & 0x04)
										Serial.print("( 3) ");
									else
										Serial.print("(  ) ");
									if(packetBuffer[4] & 0x08)
										Serial.print("( 4) ");
									else
										Serial.print("(  ) ");
									break;
								case 0x08:
										// Function Group 2
										Serial.print("FG2: ");
										if(packetBuffer[4] & 0x01)
											Serial.print("( 5) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x02)
											Serial.print("( 6) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x04)
											Serial.print("( 7) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x08)
											Serial.print("( 8) ");
										else
											Serial.print("(  ) ");
									break;
								case 0x09:
										// Function Group 3
										Serial.print("FG3: ");
										if(packetBuffer[4] & 0x01)
											Serial.print("( 9) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x02)
											Serial.print("(10) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x04)
											Serial.print("(11) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x08)
											Serial.print("(12) ");
										else
											Serial.print("(  ) ");
									break;
								case 0x15:
										// Function Group 13-20
										Serial.print("FG4: ");
										if(packetBuffer[4] & 0x01)
											Serial.print("(13) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x02)
											Serial.print("(14) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x04)
											Serial.print("(15) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x08)
											Serial.print("(16) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x10)
											Serial.print("(17) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x20)
											Serial.print("(18) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x40)
											Serial.print("(19) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x80)
											Serial.print("(20) ");
										else
											Serial.print("(  ) ");
									break;
								case 0x16:
										// Function Group 21-28
										Serial.print("FG5: ");
										if(packetBuffer[4] & 0x01)
											Serial.print("(21) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x02)
											Serial.print("(22) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x04)
											Serial.print("(23) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x08)
											Serial.print("(24) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x10)
											Serial.print("(25) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x20)
											Serial.print("(26) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x40)
											Serial.print("(27) ");
										else
											Serial.print("(  ) ");
										if(packetBuffer[4] & 0x80)
											Serial.print("(28) ");
										else
											Serial.print("(  ) ");
									break;
							}
						}
						Serial.print("{ ");
						for(i=0; i<byte_count; i++)
						{
							Serial.print(packetBuffer[i], HEX);
							Serial.print(" ");
						}
						Serial.print("}");
					}
					else
					{
						// Dumb cab packets
						uint8_t commandIndex = 0;
						if(0 == cab)
							commandIndex = 1;
						else
							commandIndex = 3;
						if(byte_count > commandIndex)
						{
							// Only process commands if we received enough bytes for there to be a command
							switch(packetBuffer[commandIndex])
							{
								case 0xC0:
								case 0xC1:
								case 0xC2:
								case 0xC3:
								case 0xC4:
								case 0xC5:
								case 0xC6:
								case 0xC7:
									Serial.print((packetBuffer[commandIndex]&0x07)/2+1);
									Serial.print(packetBuffer[commandIndex]%2?'R':'L');
									Serial.print("=");
									// ASCII
									Serial.print("\"");
									for(i=0; i<8; i++)
									{
										Serial.print((char)adjustCabBusASCII(packetBuffer[commandIndex+1+i]));
									}
									Serial.print("\"");
									Serial.print("{ ");
									for(i=0; i<byte_count; i++)
									{
										Serial.print(packetBuffer[i], HEX);
										Serial.print(" ");
									}
									Serial.print("}");
									break;
								case 0xC8:
									Serial.print("MV ");
									Serial.print(packetBuffer[commandIndex+1], HEX);
									break;
								case 0xC9:
									Serial.print("'");
									Serial.print((char)adjustCabBusASCII(packetBuffer[commandIndex+1]));
									Serial.print("' <");
									break;
								case 0xCA:
									Serial.print("'");
									Serial.print((char)adjustCabBusASCII(packetBuffer[commandIndex+1]));
									Serial.print("' >");
									break;
								case 0xD4:
									// Fast clock ratio
									Serial.print("FC ");
									Serial.print(packetBuffer[2]);
									Serial.print(":1");
									break;
								default:
									Serial.print("{ ");
									for(i=0; i<byte_count; i++)
									{
										Serial.print(packetBuffer[i], HEX);
										Serial.print(" ");
									}
									Serial.print("}");
									break;
							}
						}
					}

					Serial.print("\n");
				}

				// Reset flags
				cabBusStatus &= ~CABBUS_STATUS_SMART_CAB;
				cabBusStatus &= ~CABBUS_STATUS_DUMB_CAB;
				// Reset byte_count so the current data byte gets stored in the correct spot
				byte_count = 0;
			}
		}

		if(1 == byte_count)
		{
			// First byte of a response.  Determine the response type.
			if(data < 0x40)
				cabBusStatus |= CABBUS_STATUS_SMART_CAB;
			else
				cabBusStatus |= CABBUS_STATUS_DUMB_CAB;
		}

		// Store the byte
		packetBuffer[byte_count] = data;
		byte_count++;
	}

}


