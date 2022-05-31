import socket
import threading
from threading import Thread, Lock
import os
from time import sleep
import numpy as np 
import matplotlib.pyplot as plt 
from pylsl import StreamInlet, resolve_byprop
import utils

# Sayilan blink sayisi
blink_counter = 0
mutex = Lock()

def StartServe():

    global blink_counter
    global mutex

    localIP             = "127.0.0.1"
    localPort           = 20001
    bufferSize          = 1024
    msgFromServer       = "Hello UDP Client"
    bytesToSend         = str.encode(msgFromServer)

    # Create a datagram socket
    UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
    # Bind to address and ip
    UDPServerSocket.bind((localIP, localPort))

    print("UDP server up and listening")

    # Listen for incoming datagrams
    while(True):

        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)

        message = bytesAddressPair[0]
        address = bytesAddressPair[1]

        clientMsg = "Message from Client:{}".format(message)
        clientIP  = "Client IP Address:{}".format(address)
        
        #print(clientMsg)
        #print(clientIP)

        mutex.acquire()
        try:
            msgFromServer = "BlinkCounter: " + str(blink_counter)
            bytesToSend = str.encode(msgFromServer)
        finally:
            mutex.release()

        # Sending a reply to client
        UDPServerSocket.sendto(bytesToSend, address)

if __name__ == "__main__":

    t1 = threading.Thread(target=StartServe, name='t1')
    t1.start()
    
    class Band:
        Delta = 0
        Theta = 1
        Alpha = 2
        Beta = 3
        
    BUFFER_LENGTH = 2
    EPOCH_LENGTH = 1
    OVERLAP_LENGTH = 0.8
    SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH
    INDEX_CHANNEL = [1]

    print('Looking for an EEG stream...')
    streams = resolve_byprop('type', 'EEG', timeout=2)
    if len(streams) == 0:
        raise RuntimeError('Can\'t find EEG stream.')

    print("Start acquiring data")
    inlet = StreamInlet(streams[0], max_chunklen=12)
    eeg_time_correction = inlet.time_correction()

    info = inlet.info()
    description = info.desc()

    fs = int(info.nominal_srate())

    eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
    filter_state = None  # for use with the notch filter

    n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) /
                            SHIFT_LENGTH + 1))

    band_buffer = np.zeros((n_win_test, 4))

    # Kiyaslama icin yapilacak onceki data
    prev_data = -1

    # Kiyaslama icin yapılacak nihai data
    current_data = -1

    # hassasiyet
    sensevity = 0.30
    try:
        while True:

            """ 3.1 ACQUIRE DATA """
            # Obtain EEG data from the LSL stream
            eeg_data, timestamp = inlet.pull_chunk(
                timeout=1, max_samples=int(SHIFT_LENGTH * fs))

            # Only keep the channel we're interested in
            ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]

            # Update EEG buffer with the new data
            eeg_buffer, filter_state = utils.update_buffer(
                eeg_buffer, ch_data, notch=True,
                filter_state=filter_state)

            data_epoch = utils.get_last_data(eeg_buffer,
                                            EPOCH_LENGTH * fs)

            # Compute band powers
            band_powers = utils.compute_band_powers(data_epoch, fs)
            band_buffer, _ = utils.update_buffer(band_buffer,
                                                np.asarray([band_powers]))
            smooth_band_powers = np.mean(band_buffer, axis=0)

            alpha_metric = smooth_band_powers[Band.Alpha] / \
                smooth_band_powers[Band.Delta]

            # Degeri nihai dataya atama
            current_data = alpha_metric

            # Her adimda check olmamali gurultude de blink vermemesi icin
            check_blink = False

            # Blink oldugu zaman nihadi data hep 0 dan buyuk olmalı
            if (current_data > 0):
                
                # Eger prev data negatif ise yani ilk degeri -1 ise bir kiyas olamaz
                if (prev_data < 0):
                    prev_data = current_data
                
                # Eger prev data negatif degil yani en az 2. iterasyonda isek bir blink checki yapariz
                else:
                    check_blink = True
            
            if (check_blink):

                # Eger ki nihai data ile prev data arasi fark 0.40 dan buyuk ise blink vardir
                # Burada hassasiyet arttirip azaltilabilir
                if ((current_data - prev_data) > sensevity):

                    mutex.acquire()
                    try:
                        blink_counter += 1
                    finally:
                        mutex.release()
                    
                    print(str(blink_counter) + ")Blink Data Difference: " + str(current_data - prev_data))
                    prev_data = 0

            print('Alpha Metric: ', alpha_metric)

    except KeyboardInterrupt:
        print('Closing!')