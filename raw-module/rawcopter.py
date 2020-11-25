import socket
import time
import struct

sock = None

def connect():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(3)
    try:
        sock.connect(('localhost', 6200))
        sock.setblocking(False)
    except:
        return False
    return True

def disconnect():
    global sock
    if sock != None:
        sock.close()
        sock = None

def is_connected():
    global sock
    return sock != None

def input():
    global sock
    if sock == None:
        return None
    
    values = None
    try:
        data = sock.recv(32)
        if len(data) == 32:
            values = struct.unpack('dddd', data)
        elif len(data) == 0:
            disconnect()
    except socket.error:
        pass
    return values

def output(m1, m2, m3, m4):
    global sock
    if sock != None:
        data = struct.pack('HHHH', m1, m2, m3, m4)
        try:
            sock.sendall(data)
        except:
            return False
    else:
        return False
    
    return True