/**
 * 
 *  @file uart.c
 * 
 *  Copyright (C) 2015  Daniel Kesler <kesler.daniel@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 **/

#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define BOTHER	010000

#define VERSION	"0.03"

int set_interface_attribs (int fd, unsigned speed, unsigned custom_speed, unsigned parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tcgetattr", errno);
		return -1;
	}
	

	// output speed
	//cfsetospeed (&tty, speed);
	// input speed
	//cfsetispeed (&tty, speed);
	
	

	unsigned cflag = tty.c_cflag;
	
	if(custom_speed)
	{
		cflag &= ~CBAUD;
		cflag |= BOTHER;
	}

	tty.c_cflag = (cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	tty.c_ispeed = tty.c_ospeed = speed;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		printf ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block, unsigned timeout)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		printf ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = timeout;            // 5 = 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		printf ("error %d setting term attributes", errno);
}

struct baudrate_map {
	char* name;
	unsigned baud;
	unsigned value;
};

struct baudrate_map baudmap[] = 
{
	/* standard rates */
	{"50",B50,50},
	{"75",B75,75},
	{"110",B110,110},
	{"134",B134,134},
	{"150",B150,150},
	{"200",B200,200},
	{"300",B300,300},
	{"600",B600,600},
	{"1200",B1200,1200},
	{"1800",B1800,1800},
	{"2400",B2400,2400},
	{"4800",B4800,4800},
	{"9600",B9600,9600},
	{"19200",B19200,19200},
	{"38400",B38400,38400},
	{"57600",B57600,57600},
	{"115200",B115200,115200},
	{"230400",B230400,230400},
	/* extended rates */
	{"460800",B460800,460800},
	{"500000",B500000,500000},
	{"576000",B576000,576000},
	{"921600",B921600,921600},
	{"1000000",B1000000,1000000},
	{"1152000",B1152000,1152000},
	{"1500000",B1500000,1500000},
	{"2000000",B2000000,2000000},
	{"2500000",B2500000,2500000},
	{"3000000",B3000000,3000000},
	{"3500000",B3500000,3500000},
	{"4000000",B4000000,4000000},
	
	{0,0}
};

void uart_usage(char* appname)
{
	printf("%s -d <tty-device> -b <baud-rate> [-w -t] <data>\n", appname);
	printf("   -d <tty-device>  UART device\n");
	printf("   -b <baud-rate>   UART baud rate [default: 9600]\n");
	printf("   -w               Wait for response\n");
	printf("   -t               Wait for terminator (intrinsic -w)\n");
	printf("example:\n");
	printf("%s -d/dev/ttyS0 -b9600 \"Hello world\\n\"\n", appname);
}

char* convert_special(char *dst, size_t size, const char *src)
{
	unsigned last = 0;
	while(*src)
	{
		if(*src == '\\')
		{
			last = 1;
		}
		else
		{
			if(last)
			{
				switch(*src)
				{
					case 'n':
						*(dst++) = '\n';
						size--;
						break;
					case 'r':
						*(dst++) = '\r';
						size--;
						break;
					case 't':
						*(dst++) = '\t';
						size--;
						break;
					default:
						*(dst++) = *src;
						size--;
				}
				last = 0;
			}
			else
			{
				*(dst++) = *src;
				size--;
			}
		}
		src++;
		if(!size)
			break;
	}
	*dst = '\0';
	return dst;
}

unsigned has_terminator(const char* buf, unsigned len)
{
	while(*buf)
	{
		if(*buf == '\r')
		{
			if(len > 0)
			{
				buf++;
				len--;
				if(*buf == '\n')
				{
					return 1;
				}
			}
		}
		else if(*buf == '\n')
		{
			if(len > 0)
			{
				buf++;
				len--;
				if(*buf == '\n')
				{
					return 1;
				}
				else if(*buf == '\r')
				{
					continue;
				}
			}
		}
		buf++;
		len--;
	}
	return 0;
}

unsigned is_valid_number(char *str)
{
	while(*str)
	{
		if( !isdigit(*str) )
		{
			return 0;
		}
		str++;
	}
	
	return 1;
}

int uart_applet(int argc, char *argv[])
{
	char portname[64];
	char buffer[UART_BUFFER_SIZE];
	unsigned baud = 0, delay_value = 0;
	unsigned long custom_baudrate = 0;
	unsigned len = 0;
	unsigned param = 0;
	unsigned wait_for_response = 0;
	unsigned wait_for_terminator = 0;
	unsigned opt;
	
	if(argc < 3)
	{
		uart_usage(argv[0]);
		return -1;
	}
	
	while ((opt = getopt(argc, argv, "d:b:wt")) != -1) 
	{
		switch(opt)
		{
			case 'd':
				// get port device
				strncpy(portname, optarg, 64);
				break;
			case 'b':
				// get baud rate
				for(struct baudrate_map *bm = baudmap; bm->name; bm++)
				{
					if( strcmp(bm->name,optarg) == 0 )
					{
						baud = bm->baud;
						delay_value = bm->value;
						break;
					}
				}
				
				if( is_valid_number(optarg) )
				{
					custom_baudrate = atol(optarg);
					delay_value = custom_baudrate;
				}
				
				break;
			case 'w':
				wait_for_response = 1;
				break;
			case 't':
				wait_for_response = 1;
				wait_for_terminator = 1;
				break;
			default:
				uart_usage(argv[0]);
				return -1;					
		}
	}
	
	if (optind >= argc) {
		fprintf(stderr, "Expected argument after options\n");
		exit(EXIT_FAILURE);
	}
	
	// get user data
	convert_special(buffer, UART_BUFFER_SIZE, argv[optind]);
	len = strlen(buffer);
	
	if(!baud && !custom_baudrate)
	{
		fprintf(stderr, "unsupported baud rate\n");
		return -1;
	}
	
	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		fprintf(stderr, "error %d opening %s: %s", errno, portname, strerror (errno));
		return -1;
	}

	if(custom_baudrate)
	{
		set_interface_attribs (fd, custom_baudrate, 1, 0);
	}
	else
	{
		set_interface_attribs (fd, baud, 0, 0);
	}
	
	// set blocking with 5sec timeout
	set_blocking (fd, 1, 50);

	write (fd, buffer, len);
	
	// sleep enough to transmit the data
	usleep (len * (100000000 / delay_value) );
	
	if(wait_for_response)
	{
		do
		{
			len = read (fd, buffer, UART_BUFFER_SIZE);
	
			buffer[len] = '\0';
			printf("%s", buffer);

			if( has_terminator(buffer, len) )
				return 0;
		}
		while(wait_for_terminator);
	}
	
	return 0;
}
