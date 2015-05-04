/*
 * Copyright (C) 2014 Google, Inc.
 */

#include <nuttx/config.h>
#include <debug.h>
#include <stdio.h>
#include <stdlib.h>

#include "up_switch.h"

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int qdebug_main(int argc, char *argv[])
#endif
{
    uint32_t portid, attrid, cportid;
    uint32_t *val;
    char cmd;

    if (argc < 2) {
        dbg("qdebug: Usage:\n");
        dbg("qdebug: qdebug t port_id : list value of testing attributes.\n");
        dbg("qdebug: qdebug g port_id attribute_id [cport_id]: show value of the attribute.\n");
        return ERROR;
    } else {
        cmd = argv[1][0];
    }

    switch (cmd) {
    case 'g':
        if (argc < 4) {
            dbg("qdebug: Please enter port_id, attribute_id and cport_id.\n");
            return 0;
        }

        if (argc >= 4) {
	    portid = strtol(argv[2], 0, 10);
            attrid = strtoul(argv[3], NULL, 16);
	    cportid = strtol(argv[4], 0, 10);
	    switch_peer_getreq(portid, attrid, cportid, &val);
            dbg(" %d\n", val);
            return 0;
        }
        break;
    /*case 'l':
        if (argc == 2) {
	    show_datalink();
            return 0;
        }
        break;
    */
    case 't':
        if (argc >= 3) {
	    portid = strtol(argv[2], 0, 10);
	    uint32_t attrid_arr[] = {0xd092, 0xd093, 0x4025,0x4023 , 0x4022, 0x4021 , 0x4020, 0x007f,0x8032,0x007f,0x1568,0x1569, 0x156a, 0x1560, 0x1583, 0x1584, 0x1580, 0x15B0, 0x1571, 0xd089};
	    cportid = 0;
	    int j;
	    dbg("sizeof  %d \n",sizeof(attrid_arr));
	    for(j = 0; j < sizeof(attrid_arr)/4 ; j++){
	    switch_peer_getreq(portid, attrid_arr[j], cportid, &val);
	    dbg("attr=0x%x value=0x%x\n",attrid_arr[j], val);
	    }
            return 0;
        }
        break;
    default:
        dbg("qdebug: wrong command\n");
        return ERROR;
    }

    return 0;
}
