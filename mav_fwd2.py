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
    server = ('140.96.178.37', 8070)
    mav_master = init_mav()
    fwd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    fwd_sock.setblocking(0)
    ts = time.time()
    ts2 = time.time()
    buf = ""
    count = 0
    while True:
        data = mav_master.recv()
        if data is not None:
            buf = buf + data
            if len(buf) > 100:
                fwd_sock.sendto(buf, server)
                buf = ""
                count = count + 1
        try:
            data = fwd_sock.recv(512)
        except:
            pass
        else:
            if data is not None:
                mav_master.write(data)
        if time.time() - ts > 0.02:
            ts = time.time()
            if len(buf) > 0:
                fwd_sock.sendto(buf, server)
                buf = ""
                count = count + 1
        #if time.time() - ts2 > 1:
        #    print count, 'pps'
        #    ts2 = time.time()
        #    count = 0
