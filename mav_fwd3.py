#!/usr/bin/env python

import socket, time, select, subprocess, re, sys, serial
from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink
#from play_tune import Buzzer

class nothing(object):
    def __init__(self):
        return
    def write(self, data):
        return len(data)
    def read(self):
        return []

class empty_conn(object):
    def recv_msg(self):
        return None
    def write(self,buf):
        return

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
    mav_master, mav_modes = init_mav()
    inject_mav = mavlink.MAVLink(nothing(), mav_master.target_system, 1)

    if len(sys.argv) < 2:
        print 'no relay server_ip:port, disable mavlink forward'
        mav_relay = empty_conn()
    else:
        mav_relay = mavutil.mavlink_connection(device="udpout:"+sys.argv[1], source_system = mav_master.target_system)

    try:
        #rtk_base = serial.Serial('/dev/ttyUSB0', 57600, timeout = 0)
        rtk_base = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
    except serial.SerialException:
        rtk_base = None
    ts = time.time()
    qmi_ts = ts
    qmi_proc = None
    buf = ""
    count = 0
    arm_count = 0
    #buzzer = Buzzer()
    while True:
        cur_ts = time.time()

        msg = mav_master.recv_msg()
        if msg is not None and msg.get_type() != "BAD_DATA":
            data = msg.pack(inject_mav)
            buf = buf + data
            if len(buf) > 100:
                mav_relay.write(buf)
                buf = ""
                count = count + 1

        if rtk_base is not None:
            rtcm_msg = rtk_base.recv_msg()
            if rtcm_msg is not None and rtcm_msg.get_type() == 'GPS_RTCM_DATA':
                print 'rtcm data', rtcm_msg.len
                mav_master.mav.send(rtcm_msg)

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
                        #buzzer.play(5)
                        arm_count = arm_count + 1
                        out_msg = inject_mav.command_ack_encode(400, 1)
                        data = out_msg.pack(inject_mav)
                        buf = buf + data
                        out_msg = inject_mav.statustext_encode(5, "Arm: try again")
                        data = out_msg.pack(inject_mav)
                        buf = buf + data
            elif msg_type != "BAD_DATA":
                mav_master.mav.send(msg)

        if cur_ts - ts > 0.02:
            ts = cur_ts
            if len(buf) > 0:
                mav_relay.write(buf)
                buf = ""
                count = count + 1
        if qmi_proc is None:
            if cur_ts - qmi_ts > 5:
                try:
                    qmi_proc = subprocess.Popen(['./cellular_info.sh'], stdout=subprocess.PIPE)
                except OSError as oops_err:
                    print oops_err
                    qmi_ts = cur_ts #try again later
        else:
            if qmi_proc.poll() is not None and qmi_proc.poll() == 0:
                cell_technology = None
                cell_id = None
                txt = qmi_proc.stdout.read()
                m = re.search("Current:\n\tNetwork '([0-9a-zA-Z]+)': '-([0-9]+) dBm'", txt)
                if m is not None:
                    cell_technology = m.group(1).lower()
                    cell_rssi = int(m.group(2))
                    print 'network', cell_technology, cell_rssi
                m = re.search("Cell ID: '([0-9]+)'\n\t\tMCC: '([0-9]+)'\n\t\tMNC: '([0-9]+)'\n\t\tTracking Area Code: '([0-9]+)'", txt)
                if m is not None:
                    cell_id = int(m.group(1))
                    cell_mcc = int(m.group(2))
                    cell_mnc = int(m.group(3))
                    cell_tac = int(m.group(4))
                    print 'cell', cell_id, cell_mcc, cell_mnc, cell_tac
                if cell_technology is not None and cell_id is not None:                
                    msg = inject_mav.cellular_status_encode(cell_technology, cell_rssi, cell_mcc, cell_mnc, cell_tac, cell_id)
                    data = msg.pack(inject_mav)
                    buf = buf + data                
                qmi_ts = time.time()
                qmi_proc = None

if __name__ == "__main__":
    my_main()

