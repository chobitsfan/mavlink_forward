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
    mav_master = mavutil.mavlink_connection(device="/dev/ttyAMA0", baud=115200, source_system=255)
    print "Waiting for APM heartbeat"
    while True:
        hb = mav_master.recv_match(type='HEARTBEAT', blocking=True)
        if hb.type != mavutil.mavlink.MAV_TYPE_GCS:
            print "Heartbeat from APM system", mav_master.target_system
            break
    mav_modes = mav_master.mode_mapping()
    return (mav_master, mav_modes)

def my_main():
    if len(sys.argv) < 2:
        print 'server_ip:port required'
        sys.exit()
    mav_master, mav_modes = init_mav()
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
    gcs_hb_ts = 0
    Normal, Hold, Recall, Failsafe = range(4)
    status = Normal
    recall_point = None
    recall_point_ts = 0
    while True:
        cur_ts = time.time()

        msg = mav_master.recv_msg()
        if msg is not None and msg.get_type() != "BAD_DATA":
            if msg.get_type() == "GLOBAL_POSITION_INT" and status == Normal and mav_master.flightmode in ["AUTO", "GUIDED"] and cur_ts - recall_point_ts > 5:
                recall_point_ts = cur_ts
                recall_point = (msg.lat, msg.lon, msg.alt)
                print 'record recall point', recall_point
            elif msg.get_type() == "HEARTBEAT":
                if status == Hold and mav_master.flightmode != "BRAKE":
                    mav_master.set_mode(mav_modes["BRAKE"])
                elif status == Recall:
                    if mav_master.flightmode == "GUIDED":
                        mav_master.mav.set_position_target_global_int_send(0, mav_master.target_system, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_INT, 0b0000111111111000, recall_point[0], recall_point[1], recall_point[2] / 1000.0, 0, 0, 0, 0, 0, 0, 0, 0)
                    else:
                        mav_master.set_mode(mav_modes["GUIDED"])
                elif status == Failsafe and mav_master.flightmode != "LAND":
                    mav_master.set_mode(mav_modes["LAND"])
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
            elif msg_type == "HEARTBEAT":
                gcs_hb_ts = time.time()
                mav_master.mav.send(msg)
            elif msg_type != "BAD_DATA":
                mav_master.mav.send(msg)

        if status == Normal:
            if mav_master.flightmode in ["AUTO", "GUIDED"] and gcs_hb_ts != 0 and cur_ts - gcs_hb_ts > 5:
                print "Hold"
                status = Hold
                mav_master.set_mode(mav_modes["BRAKE"])
        elif status == Hold:
            if cur_ts - gcs_hb_ts > 10:
                print "Recall"
                status = Recall
                if recall_point is not None:
                    mav_master.set_mode(mav_modes["GUIDED"])
            elif cur_ts - gcs_hb_ts < 1:
                print "Normal"
                status = Normal
        elif status == Recall:
            if cur_ts - gcs_hb_ts > 20:
                print "Failsafe"
                status = Failsafe
                mav_master.set_mode(mav_modes["LAND"])
            elif cur_ts - gcs_hb_ts < 1:
                print "Hold"
                status = Hold
                mav_master.set_mode(mav_modes["BRAKE"])
        elif status == Failsafe:
            if cur_ts - gcs_hb_ts < 1:
                print "Hold"
                status = Hold
                mav_master.set_mode(mav_modes["BRAKE"])

        if cur_ts - ts > 0.02:
            ts = cur_ts
            if len(buf) > 0:
                mav_relay.write(buf)
                buf = ""
                count = count + 1
        if cur_ts - ts2 > 1:
            #print count, 'pps'
            ts2 = cur_ts
            count = 0
        if qmi_proc is None:
            if cur_ts - qmi_ts > 5:
                try:
                    qmi_proc = subprocess.Popen(['/usr/bin/qmicli', '-d','/dev/cdc-wdm0','--nas-get-signal-strength'], stdout=subprocess.PIPE)
                except OSError:
                    qmi_ts = cur_ts #try again later
        else:
            if qmi_proc.poll() is not None and qmi_proc.poll() == 0:
                txt = qmi_proc.stdout.read()
                s = txt.find('Current:\n\tNetwork')
                if s > -1:
                   m = re.match("'([0-9a-zA-Z]+)': '-([0-9]+) dBm'", txt[s+18:])
                   if m is not None:
                       cel_technology = m.group(1).lower()
                       cel_rssi = m.group(2)
                       #print 'celluar_status', cel_technology, cel_rssi
                       msg = inject_mav.cellular_status_encode(cel_technology, int(cel_rssi))
                       data = msg.pack(inject_mav)
                       buf = buf + data
                qmi_ts = time.time()
                qmi_proc = None

if __name__ == "__main__":
    my_main()

