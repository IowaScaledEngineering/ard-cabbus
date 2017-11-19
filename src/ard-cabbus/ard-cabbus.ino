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

void processPacket(uint8_t *packetBuffer, uint8_t byte_count)
{
	uint8_t i;
	uint16_t cmd;

	uint8_t cab = packetBuffer[0] & 0x3F;

	if(byte_count > 3)  // More than just a dumb cab response or AIU response
	{
		Serial.print("T");
		if(cab)
			Serial.print(cab);
		else
			Serial.print('*');
		Serial.print(":");

		// Determine type of response.
		// Relying on smart cab commands (byte 3) being less than dumb cab commands (start at 0x40), although Cab Bus spec allows smart cab commands up to 0x7F (reserved above 0x19)
		if( (0x4E == packetBuffer[1]) && (0x50 <= packetBuffer[2]) && (packetBuffer[2] <= 0x7F) && !(0x80 & packetBuffer[3]) )
		{
			// OPS programming
			// 0x4E + {0x50 - 0x7F} could also be a dumb cab response.
			// However, the following dumb cab command (byte 3) will have the MSB set
			uint16_t cvAddress = (((uint16_t)packetBuffer[2] & 0x0F) << 6) + ((packetBuffer[3] & 0x7E) >> 1);
			uint8_t cvData = ((packetBuffer[3] & 0x01) << 7) + (packetBuffer[4] & 0x7F);
			Serial.print("CV");
			Serial.print(cvAddress);
			Serial.print("=");
			Serial.print(cvData);
		}
		else if( (0x4E == packetBuffer[1]) && (0x18 <= packetBuffer[2]) && (packetBuffer[2] <= 0x19) )// && !(0x80 & packetBuffer[3]) )
		{
			// Cab Memory Access
			// 0x4E + {0x18 - 0x19} could also be a dumb cab response.
			// However, the following dumb cab command (byte 3) will have the MSB set
			uint8_t op1 = (packetBuffer[3] & 0x7E) >> 1;
			uint8_t op2 = ((packetBuffer[3] & 0x01) << 7) + (packetBuffer[4] & 0x7F);
			if(0x18 == packetBuffer[2])
			{
				// Memory Address Pointer
				Serial.print("Ad=");
				Serial.print(op1);
				Serial.print(",Pg=");
				Serial.print(op2);
			}
			else
			{
				// Memory Read/Write
				switch(op1)
				{
					case 0x00:
						Serial.print("Wr=0x");
						Serial.print(op2, HEX);
						break;
					case 0x01:
						Serial.print("Wr=0x");
						Serial.print(op2+128, HEX);
						break;
					case 0x02:
						Serial.print("Read");
						Serial.print(op2);
						break;
					case 0x03:
						Serial.print("RdAIU(");
						Serial.print(op2);
						Serial.print(")");
						break;
				}
				if(byte_count > 5)
				{
					// Response to a read
					switch(packetBuffer[7] & 0x30)
					{
						case 0x00:
							Serial.print(" Succ ");
							break;
						case 0x10:
							Serial.print(" Fail ");
							break;
						case 0x20:
						case 0x30:
							Serial.print(" ???? ");
							break;
					}
					switch(packetBuffer[6])
					{
						case 0xD8:
							Serial.print( ((packetBuffer[7] & 0x03) << 6) + (packetBuffer[8] & 0x3F), HEX);
							break;
						case 0xD9:
							Serial.print( ((packetBuffer[7] & 0x0F) << 4) + (packetBuffer[8] & 0x0F), HEX);
							Serial.print(" ");
							Serial.print( ((packetBuffer[8] & 0x30) << 2) + (packetBuffer[9] & 0x3F), HEX);
							break;
						case 0xDA:
							Serial.print( ((packetBuffer[7] & 0x0F) << 4) + (packetBuffer[8] & 0x0F), HEX);
							Serial.print(" ");
							Serial.print( ((packetBuffer[8] & 0x30) << 2) + (packetBuffer[9] & 0x3F), HEX);
							Serial.print(" ");
							Serial.print( ((packetBuffer[10] & 0x0F) << 4) + (packetBuffer[11] & 0x0F), HEX);
							Serial.print(" ");
							Serial.print( ((packetBuffer[11] & 0x30) << 2) + (packetBuffer[12] & 0x3F), HEX);
							break;
					}
				}
			}
		}
		else if(packetBuffer[3] < 0x40)
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
						Serial.print("/128 ");
						break;
					case 0x04:
						// Forward 128 speed command
						Serial.print("Fwd: ");
						Serial.print(packetBuffer[4]);
						Serial.print("/128 ");
						break;
					case 0x05:
						// Reverse E-stop
						Serial.print("ESTOP (Rev) ");
						break;
					case 0x06:
						// Forward E-stop
						Serial.print("ESTOP (Fwd) ");
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
		}
		else
		{
			// Dumb cab packets
			uint8_t commandIndex = 3;
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
					case 0xCE:
						Serial.print("CRS-");
						break;
					case 0xCF:
						Serial.print("CRS+");
						break;
					case 0xD4:
						// Fast clock ratio
						Serial.print("CLK ");
						Serial.print((packetBuffer[commandIndex+1] & 0x3F));
						Serial.print(":1");
						break;
				}
			}
		}
		Serial.print(" { ");
		for(i=0; i<byte_count; i++)
		{
			if( (0x80 == packetBuffer[0]) && (1 <= i) && (i <= 2) )
				Serial.print("-");
			else
				Serial.print(packetBuffer[i], HEX);
			Serial.print(" ");
		}
		Serial.println("}");
	}
}

void loop()
{
	uint8_t i;
	uint16_t data;

	uint8_t byte_count = 0;
	uint8_t packetBuffer[PKT_BUFFER_SIZE];

while(1)
{
	if(rxBufferDepth() > 1)
	{
		data = rxBufferPop(0);

		// Dumb Cab:
		// byte 0: --> ping
		// byte 1: <-- response byte 1
		// byte 2: <-- response byte 2
		// byte 3: --> Command byte 1 (11xx xxxx)
		// byte 4: --> Command byte 2 or next ping (sometimes doesn't follow the 11xx xxxx rule)
		// byte n: --> More command bytes or next ping (sometimes doesn't follow the 11xx xxxx rule, e.g. command 0xCB)
		// Note: Bytes 4-n could look like pings but are conditional on the command byte 1 value
		//
		// Smart Cab:
		// byte 0: --> ping
		// byte 1: <-- address 1
		// byte 2: <-- address 2
		// byte 3: <-- Command
		// byte 4: <-- Value
		// byte 5: <-- Checksum
		// Note: Bytes 4-5 are allowed to have the MSB set

		// Does the byte have the characteristics of a ping (10xx xxxx)?
		if((data & 0xC0) == 0x80)
		{
			// The byte might be a ping, but we need to check the exceptions
			uint8_t cmd = packetBuffer[3];
			if(
				!( (0x15 <= cmd) && (cmd <= 0x16) && (4 <= byte_count) && (byte_count <= 5)  ) &&  // Ignore bytes 4 & 5 of smart cab FN13-28 responses (Note 1)
				!( (0xCB == cmd)                  && (4 <= byte_count) && (byte_count <= 12) ) &&  // Ignore 9 bytes following dumb cab command 0xCB
				!( (0xC0 <= cmd) && (cmd <= 0xC7) && (4 <= byte_count) && (byte_count <= 11) ) &&  // Ignore 8 bytes following dumb cab commands 0xC0 to 0xC7, special chars are not escaped (Note 2)
				!( (0xC8 == cmd)                  && (4 == byte_count)                       ) &&  // Ignore data for command 0xC8 (unverified, based on example from spec)
				!( (0xCC == cmd)                  && (4 == byte_count)                       )     // Ignore data for command 0xCC (?)
			)
			// Note 1: This could catch some cab memory access and OPS programming packets, too, but these are properly escaped so it shouldn't ever get here.
			// Note 2: Although ASCII chars are escaped in the 8 char writes, special chars are not (e.g. EXPN display for FN10 and FN20).
			//         Dumb cabs appear to be fooled by this and cause collisions, though the data appears to still get through.  Observed collisions on scope.
			{
				// Must be a ping, so handle it
				processPacket(packetBuffer, byte_count);

				// Reset byte_count so the current data byte gets stored in the correct (index = 0) spot
				byte_count = 0;
			}
		}

		if(1 == byte_count)
		{
			// First byte after ping
			if(0x80 == packetBuffer[0])
			{
				// Broadcast command, skip 2 bytes so it aligns with normal commands
				packetBuffer[byte_count++] = 0;
				packetBuffer[byte_count++] = 0;
			}
		}

		// Store the byte
		packetBuffer[byte_count] = data;
		byte_count++;
	}
}
}


