#!/usr/bin/env python

import socket, time, select, subprocess, re, sys
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink
from play_tune import Buzzer

class nothing(object):
    def __init__(self):
        return
    def write(self, data):
        return len(data)
    def read(self):
        return []

def init_mav():
    mav_master = mavutil.mavlink_connection(device="/dev/ttyAMA0", baud=921600, source_system=255)
    print "Waiting for APM heartbeat"
    while True:
        hb = mav_master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            print "Heartbeat from APM system", mav_master.target_system
            break
    return mav_master

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print 'server_ip:port required'
        sys.exit()
    mav_master = init_mav()
    inject_mav = mavlink.MAVLink(nothing(), mav_master.target_system, 1)
    mav_relay = mavutil.mavlink_connection(device="udpout:"+sys.argv[1], source_system = mav_master.target_system)
    ts = time.time()
    ts2 = ts
    qmi_ts = ts
    qmi_proc = None
    buf = ""
    count = 0
    arm_count = 0
    buzzer = Buzzer()
    while True:
        msg = mav_master.recv_msg()
        if msg is not None and msg.get_type() != "BAD_DATA":
            data = msg.pack(inject_mav)
            buf = buf + data
            if len(buf) > 100:
                mav_relay.write(buf)
                buf = ""
                count = count + 1

        msg = mav_relay.recv_msg()
        if msg is not None:
            msg_type = msg.get_type()
            if msg_type == "PING":
                print 'recv ping'
                out_msg = inject_mav.ping_encode(msg.time_usec, msg.seq, msg.get_srcSystem(), msg.get_srcComponent())
                data = out_msg.pack(inject_mav)
                buf = buf + data
            elif msg_type == "COMMAND_LONG" and msg.command == 400 and msg.target_system == mav_master.target_system: #ARM_DSARM
                if msg.param1 == 0:
                    print 'recv disarm'
                    arm_count = 0
                    mav_master.mav.send(msg)
                else:
                    print 'recv arm'
                    if arm_count > 1:
                        arm_count = 0
                        mav_master.mav.send(msg)
                    else:
                        buzzer.play(5)
                        arm_count = arm_count + 1
                        out_msg = inject_mav.command_ack_encode(400, 1)
                        data = out_msg.pack(inject_mav)
                        buf = buf + data
                        out_msg = inject_mav.statustext_encode(5, "Arm: try again")
                        data = out_msg.pack(inject_mav)
                        buf = buf + data
            elif msg_type != "BAD_DATA":
                mav_master.mav.send(msg)

        cur_ts = time.time()
        if cur_ts - ts > 0.02:
            ts = cur_ts
            if len(buf) > 0:
                mav_relay.write(buf)
                buf = ""
                count = count + 1
        if cur_ts - ts2 > 1:
            print count, 'pps'
            ts2 = cur_ts
            count = 0
        if qmi_proc is None:
            if cur_ts - qmi_ts > 5:
                qmi_proc = subprocess.Popen(['/usr/bin/qmicli', '-d','/dev/cdc-wdm0','--nas-get-signal-strength'], stdout=subprocess.PIPE)
        else:
            if qmi_proc.poll() is not None and qmi_proc.poll() == 0:
                txt = qmi_proc.stdout.read()
                s = txt.find('Current:\n\tNetwork')
                if s > -1:
                   m = re.match("'([0-9a-zA-Z]+)': '-([0-9]+) dBm'", txt[s+18:])
                   if m is not None:
                       cel_technology = m.group(1).lower()
                       cel_rssi = m.group(2)
                       print 'celluar_status', cel_technology, cel_rssi
                       msg = inject_mav.cellular_status_encode(cel_technology, int(cel_rssi))
                       data = msg.pack(inject_mav)
                       buf = buf + data
                qmi_ts = time.time()
                qmi_proc = None

