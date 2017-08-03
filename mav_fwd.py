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
    ts = time.time()
    buf = ""
    poller = select.poll()
    poller.register(mav_master.port.fileno(), select.POLLIN | select.POLLPRI)
    poller.register(fwd_sock.fileno(), select.POLLIN | select.POLLPRI)
    fd_to_obj = {mav_master.port.fileno(): mav_master, fwd_sock.fileno():fwd_sock}
    while True:
        try:
            events = poller.poll(10)
        except KeyboardInterrupt:
            break
        for fd, flag in events:
            target = fd_to_obj[fd]
            if target == mav_master:
                data = mav_master.recv()
                if data is not None:
                    #print len(data)
                    buf = buf + data
                if len(buf) > 500:
                    print len(buf)
                    fwd_sock.sendto(buf, server)
                    buf = ""
            else:
                data = fwd_sock.recv(512)
                if data is not None:
                    mav_master.write(data)
        if time.time() - ts > 0.001:
            ts = time.time()
            if len(buf) > 0:
                print len(buf)
                fwd_sock.sendto(buf, server)
                buf = ""
