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
uint8_t packetBufferIndex = 0;

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

void setup()
{
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

	rxBufferInitialize();

	Serial.begin(57600);
	serialInit();
}

void loop()
{
	uint16_t data, cmd;
	uint8_t i, speed;
	uint16_t locomotive;

	if(rxBufferDepth() > 1)
	{
		data = rxBufferPop(0);

		if((data & 0xC0)==0x80)
		{
			// It's a ping
			if(packetBufferIndex > 1)
			{
				Serial.print("T");
				uint8_t cab = packetBuffer[0] & 0x7F;
				if(cab)
					Serial.print(packetBuffer[0] & 0x7F);
				else
					Serial.print('*');
				Serial.print(": ");
				if(packetBuffer[1] > 0xC0)
				{
					// Dumb cab command
					switch(packetBuffer[1])
					{
						case 0xC1:
							Serial.print("LCD: ");
							for(i=0; i<8; i++)
							{
								uint8_t chr = packetBuffer[2+i];
								if(chr & 0x20)
									chr &= 0x3F;  // Clear bit 6 & 7
								else
									chr &= 0x7F;  // Clear only bit 7
								Serial.print((char)chr);
							}
							break;
						case 0xD4:
							Serial.print("FC Ratio = ");
							Serial.print(packetBuffer[2]);
							break;
						default:
							for(i=0; i<packetBufferIndex; i++)
							{
								Serial.print(packetBuffer[i], HEX);
								Serial.print(" ");
							}
							break;
					}
					Serial.print("\n");
				}
				else
				{
					// Response (assumes smart cabs only for now)
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
					Serial.print(" {0x");
					for(i=1; i<=5; i++)
					{
						Serial.print((packetBuffer[i]&0xF0)>>4, HEX);
						Serial.print(packetBuffer[i]&0x0F, HEX);
						if(i<5)
						{
							Serial.print(" 0x");
						}
						else
						{
							Serial.print("}");
						}
					}
					Serial.print("\n");
/*					for(i=0; i<PKT_BUFFER_SIZE; i++)*/
/*					{*/
/*						Serial.print(packetBuffer[i], HEX);*/
/*						Serial.print(" ");*/
/*					}*/
/*					Serial.print("\n");*/
				}
			}
			packetBufferIndex = 0;
			// Clear packet buffer
			for(i=0; i<PKT_BUFFER_SIZE; i++)
			{
				packetBuffer[i] = 0;
			}
/*			// If next byte in RxBuffer is not another cmd with MSB set, then start filling buffer in prep for getting a response.*/
/*			if(!(rxBufferPop(1) & 0x80))  // Snoop, but don't pop it yet (will be done in next loop)*/
/*			{*/

				// Start filling buffer again
				packetBuffer[packetBufferIndex] = data;
				if(packetBufferIndex < (PKT_BUFFER_SIZE-1))
					packetBufferIndex++;

/*			}*/
		}
		else
		{
			//  Not a ping
			packetBuffer[packetBufferIndex] = data & 0xFF;
			if(packetBufferIndex < (PKT_BUFFER_SIZE-1))
				packetBufferIndex++;
		}
	}
}


