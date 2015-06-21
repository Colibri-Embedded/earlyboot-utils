/**
 * 
 *  @file main.c
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

#include "common.h"
#include "uart.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <libgen.h>

struct applet_t applets[] = {
	{"ebuart", uart_applet},
	{0,0}
};

int main(int argc, char *argv[])
{
	char* appname = basename(argv[0]);
	
	for(struct applet_t *app = applets; app->name && app->main; app++)
	{
		if( strcmp(app->name, appname) == 0 )
		{
			return app->main(argc, argv);
		}
	}
	return 0;
}
