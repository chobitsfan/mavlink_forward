#!/usr/bin/env python

import socket, time, select
from pymavlink import mavutil

def init_mav():
    mav_master = mavutil.mavlink_connection(device="/dev/ttyAMA0", baud=921600, source_system=250)
    print "Waiting for APM heartbeat"
    while True:
        hb = mav_master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            print "Heartbeat from APM system", mav_master.target_system
            break
    return mav_master

if __name__ == "__main__":
    mav_master = init_mav()
    mav_relay = mavutil.mavlink_connection(device="udpout:140.96.178.37:8070", source_system=250)
    ts = time.time()
    ts2 = time.time()
    buf = ""
    count = 0
    while True:
        data = mav_master.recv()
        if data is not None:
            buf = buf + data
            if len(buf) > 100:
                mav_relay.write(buf)
                buf = ""
                count = count + 1

        msg = mav_relay.recv_msg()
        if msg is not None:
            print msg.get_type()
            if msg.get_type() != "BAD_DATA":
                mav_master.mav.send(msg)

        if time.time() - ts > 0.02:
            ts = time.time()
            if len(buf) > 0:
                mav_relay.write(buf)
                buf = ""
                count = count + 1
        #if time.time() - ts2 > 1:
        #    print count, 'pps'
        #    ts2 = time.time()
        #    count = 0
