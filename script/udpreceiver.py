import socket
import datetime
import time

def nowstr():
    return datetime.datetime.fromtimestamp(
        int(time.time())
    ).strftime('[%Y-%m-%d,%H:%M:%S]')


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

UDP_IP = "192.168.0.159"
UDP_PORT = 6100

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

lat, lon, alt = 0,0,0

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    msg = data.split(',')

    sock.sendto("ACK", addr)

    try:
        if msg[0] == "LOC":
            lat = float(msg[1])
            lon = float(msg[2])
            alt = float(msg[3])
            print nowstr() + bcolors.OKBLUE  + " Location : ", lat, lon, alt, bcolors.ENDC
        elif msg[0] == "MOU":
            mx = float(msg[1])
            my = float(msg[2])
            print nowstr() + bcolors.OKGREEN + " X/Y : ", mx, my, bcolors.ENDC
        else:
            print nowstr() + bcolors.WARNING + " Received : ", data, bcolors.ENDC
    except:
        print nowstr() + bcolors.FAIL + " Error reading MSG" + bcolors.ENDC
