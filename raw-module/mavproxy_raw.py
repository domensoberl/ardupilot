#!/usr/bin/env python
import os
import os.path
import sys
from pymavlink import mavutil
import errno
import time
import socket
import struct

from MAVProxy.modules.lib import mp_module
from MAVProxy.modules.lib import mp_util
from MAVProxy.modules.lib import mp_settings


class raw(mp_module.MPModule):
    def __init__(self, mpstate):
        super(raw, self).__init__(mpstate, "raw", "module raw")

        self.raw_settings = mp_settings.MPSettings(
            [ ('verbose', bool, False),
          ])
        self.add_command('raw', self.cmd_raw, "raw module", ['state','output'])

        self.pitch = 0
        self.roll = 0
        self.yaw = 0
        self.altitude = 0
        self.dt = 0
        self.freq = 0
        self.attitude_received = False
        self.altitude_received = False
        self.timestamp = time.time()

        # Socket
        self.port = 6200
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(('localhost', self.port))
        self.sock.setblocking(False)
        self.sock.listen()
        self.conn = None
        print("Raw module is listening at port %d." % (self.port))
    def usage(self):
        return "Usage: raw <state|output>"

    def print_state(self):
        print("pitch = %.4f, roll = %.4f yaw = %.4f, altitude = %.2f" % (self.pitch, self.roll, self.yaw, self.altitude))

    def output(self, args):
        if len(args) < 5:
            print("Usage: output m1 m2 m3 m4")
            return
        self.send_action(args[1], args[2], args[3], args[4])

    def send_action(self, m1, m2, m3, m4):
        self.master.mav.command_long_send(
                self.settings.target_system,
                self.settings.target_component,
                mavutil.mavlink.MAV_CMD_DO_RAW_OUTPUT,
                0, # confirmation
                float(m1),
                float(m2),
                float(m3),
                float(m4),
                0, 0, 0) # params

    def cmd_raw(self, args):
        if len(args) == 0:
            print(self.usage())
        elif args[0] == "state":
            self.print_state()
        elif args[0] == "output":
            self.output(args)
        else:
            print(self.usage())

    def idle_task(self):
        if self.conn == None:
            try:
                self.conn, self.addr = self.sock.accept()
                self.conn.setblocking(False)
                print("Raw client ", self.addr, "connected.")
            except:
                self.conn = None
        else:
            try:
                conn, addr = self.sock.accept()
                print("Raw client ", self.addr, "refused (only one client allowed).")
                conn.close()
            except:
                pass

        if self.conn != None:
            try:
                data = self.conn.recv(8)
                if len(data) == 8:
                    (m1, m2, m3, m4) = struct.unpack('HHHH', data)
                    self.send_action(m1, m2, m3, m4)
                elif len(data) == 0:
                    print("Raw client ", self.addr, "disconnected.")
                    self.conn.close()
                    self.conn = None
                else:
                    print("Raw module received a packet of illegal size from the client.")
            except socket.error:
                pass

    def mavlink_packet(self, m):
        if m.get_type() == 'ATTITUDE':
            self.pitch = m.pitch
            self.roll = m.roll
            self.yaw = m.yaw
            self.attitude_received = True

        if m.get_type() == 'GLOBAL_POSITION_INT':
            self.altitude = m.relative_alt / 1000.0
            self.altitude_received = True

        if self.attitude_received and self.altitude_received:
            # Update the state.
            now = time.time()
            self.dt = now - self.timestamp
            self.timestamp = now
            self.attitude_received = False
            self.altitude_received = False
            
            # Send the data to the client
            if self.conn != None:
                data = struct.pack('dddd', self.pitch, self.roll, self.yaw, self.altitude)
                try:
                    self.conn.sendall(data)
                except:
                    print("Raw module failed sending data to the client.")

    def unload(self):
        if self.conn != None:
            self.conn.close()
        self.sock.close()

def init(mpstate):
    return raw(mpstate)
