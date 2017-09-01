import socket
import datetime
import time
import threading
import numpy as np
from scipy import signal
import matplotlib.pyplot as plt
from rtlsdr import *

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
height, width = 0,0
spacing = 8
# phi = 0

#plt.ion()

def genWave(size):
    # global phi
    # theta = np.arange(phi,2*2*np.pi + phi,2* spacing* 2*np.pi/float(size))
    # wave = np.sin(theta)
    # phi = phi + 0.1

    inbytes = np.fromfile("/tmp/samples.bin", dtype=np.uint8)
    norm = np.empty(len(inbytes)//2)
    norm.fill(128)
    samples = np.empty(len(inbytes)//2, 'complex')
    samples.real = (inbytes[::2] - norm)/ 128.0
    samples.imag = (inbytes[1::2] - norm)/ 128.0

    start = time.time()
    f, Pxx_den = signal.welch(samples, 2.4e6, nperseg=1024)
    N = len(Pxx_den)/2
    w =  10*np.log10(1e6*np.concatenate( (Pxx_den[N:], Pxx_den[1:N]) ) )
    wave = w  / max(abs(w))

    #plt.plot(np.arange(0,len(wave)), wave, hold=False )

    #plt.pause(0.005)


    s = ",".join(str(x) for x in list(wave) )
    return s

genWave(19)

def reader(run_event):
    # DEVICE SETTINGS
    sample_rate = int(1.024e6)
    center_freq = int(1575.42e6)
    center_freq_offset = 250e3         # Offset to capture at
    sdr = RtlSdr()
    sdr.sample_rate = sample_rate # Hz
    sdr.center_freq = center_freq - center_freq_offset # Hz
    sdr.freq_correction = 60   # PPM
    sdr.gain = 'auto'

    print("IM reader")
    while run_event.is_set():
        print("IM RUNNIG")
        sock.sendto( "PSD," + genWave(1440) , addr)

# run_event = threading.Event()
# run_event.set()
# t1 = threading.Thread(target=reader, args=(run_event))

# try:
print("STARTING UDP SERVER")
try:
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
            elif msg[0] == "PSD":
                wave = genWave(msg[1])
                sock.sendto( "PSD," + wave , addr)
                print nowstr() + bcolors.OKGREEN + " PSD ", msg[1] ," requested", bcolors.ENDC
            elif msg[0] == "SCR":
                height = int(msg[1]);
                width = int(msg[2]);
                print nowstr() + bcolors.OKGREEN + " SIZE : ", str(width) + "x" + str(height), bcolors.ENDC
            else:
                print nowstr() + bcolors.WARNING + " Received : ", data, bcolors.ENDC
        except:
            print nowstr() + bcolors.FAIL + " Error reading MSG" + bcolors.ENDC

except KeyboardInterrupt:
        run_event.clear()
        t1.join()
        print ("threads successfully closed")
