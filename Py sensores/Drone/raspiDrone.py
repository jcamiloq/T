# Python3 Script
# Developed by Camilo Quintero - October 2018
# Universidad del Valle, Cali, Colombia

# Parallel execution of the two specs: https://stackoverflow.com/questions/7207309/python-how-can-i-run-python-functions-in-parallel

from multiprocessing import Queue
import seabreeze.spectrometers as sb
import numpy as np
import time
import datetime
from time import sleep
import STS_functions as sts  # This script was created in order to ease the readability of the developed script
import time
import serial
import argparse
import itertools
from itertools import *
global titleV, titleN, inte, cap, NIR_S, VIS_S
#import seabreeze.pyseabreeze.interfaces.communication as tuptm
#import threading
parser = argparse.ArgumentParser()
parser.add_argument("--connect", dest="conect", help="Número de capturas")
results = parser.parse_args()
datanum = results.conect
print(str(datanum))
# Creación de archivos para guardar los datos capturados
saveFile = open("/home/pi/Desktop/Data/DataNIR%s.txt" % (datanum), 'w')
# saveFile.write('Inicio')
saveFile.close()

saveFile = open("/home/pi/Desktop/Data/DataVIS%s.txt" % (datanum), 'w')
# saveFile.write('Inicio')
saveFile.close()

titleV = titleN = []
inte = 20
cap = 5
NIR_S = "S09067"
VIS_S = "S09645"


def file():
    saveFile = open("/home/pi/Desktop/Data/DataNIR%s.txt" % (fileid), 'w')
    #saveFile.write(str(inte) + " " + str(cap))
    saveFile.close()

    saveFile = open("/home/pi/Desktop/Data/DataVIS%s.txt" % (fileid), 'w')
    #saveFile.write(str(inte) + '' + str(cap))
    saveFile.close()

# recepción de espectros de referencia


def read_NIR():
    spectrum_NIR = np.array(spec_NIR.spectrum())
    queue.put(spectrum_NIR)


def read_VIS():
    spectrum_VIS = np.array(spec_VIS.spectrum())
    queue.put(spectrum_VIS)
    # queue.put(spectrum_VIS)

#global s_read


def w_file_NIR(arr1, arr2):
    
    global file_counter_NIR, titleN
        #titleN = ('Captura%s' % str(file_counter_NIR))
        # print(titleN)
    titleN = [i for i in itertools.chain(titleN, arr2)]
        #titleN = titleN + intensities_NIR
        # appendFile = open("/home/pi/Desktop/Data/DataNIR%s.txt" % (fileid), 'a')
        # appendFile.write('\n' + str(datetime.datetime.now()))
        # saveFile.close()

    for i in range(len(arr1)):
        appendFile = open("/home/pi/Desktop/Data/DataNIR%s.txt" % (fileid), 'a')
        appendFile.write('\n' + str(arr2[i]))
            #appendFile.write('\n' + str(arr1[i]) + " " + str(arr2[i]))
        appendFile.close()

    file_counter_NIR = file_counter_NIR + 1


def writefileN(arr1):
    for j in range(0, num_cap):

        title2n = titleN[0:1024 + 1024 * j]
        for i in range(len(arr1)):
            appendFile = open("/home/pi/Desktop/Data/DataNIR%s.txt" % (fileid), 'a')
            appendFile.write('\n' + str(arr1[i]) + " " + str(title2n[i]))
            #appendFile.write('\n' + str(arr1[i]) + " " + str(arr2[i]))
            appendFile.close()


def w_file_VIS(arr1, arr2):
    
    global file_counter_VIS, titleV
        # titleV = ('Captura número ' + str(file_counter_VIS))
        #titleV = ('Captura%s' % str(file_counter_VIS))
    titleV = [i for i in itertools.chain(titleV, arr2)]
        #titleV = titleV + arr2
        # appendFile = open("/home/pi/Desktop/Data/DataVIS%s.txt" % (fileid), 'a')
        # appendFile.write('\n' + str(datetime.datetime.now()))
    for i in range(len(arr1)):
        appendFile = open("/home/pi/Desktop/Data/DataVIS%s.txt" % (fileid), 'a')
        appendFile.write('\n' + str(arr2[i]))
        appendFile.close()

    # saveFile.close()
    file_counter_VIS = file_counter_VIS + 1


def writefileV(arr1):
    for j in range(0, num_cap):
        #titleV = ('Captura%s' % str(j))
        title2v = titleN[0:1024 + 1024 * j]
        for i in range(len(arr1)):
            appendFile = open("/home/pi/Desktop/Data/DataVIS%s.txt" % (fileid), 'a')
            appendFile.write('\n' + str(arr1[i]) + " " + str(title2v[i]))
            appendFile.close()


# def main():
#     s_read = 1
#     chain = 'l'
#     ser.write(chain.encode())
#     print('Listo')
#     ser.close()
def main2(tint, cap):
    sleep(3)
    print(str(tint) + " " + str(cap))
    chain = 'r'
    ser.write(chain.encode())
    print('LiSto')


def main(tint, cap):
    #tint = 200
    #cap = 5
    global spec_NIR, spec_VIS, wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS, queue, out
    try:
        spec_NIR = sts.setup_Spec(sb.Spectrometer.from_serial_number(NIR_S), tint * 10000, cap)  # Serial, integration time (us), scans per avg.
    except:
        print("NIR serial number not found")
        #NIR_S = input("Insert NIR serial number" +  "\n")
        #main2(tint, cap)
        #return
    try:
        spec_VIS = sts.setup_Spec(sb.Spectrometer.from_serial_number(VIS_S), tint * 10000, cap)
    except:
        print("VIS serial number not found")
        #VIS_S = input("Insert VIS serial number" +  "\n")
        #main2(tint, cap)
        #return
    #out = []
    try:
        start_time_A = time.time()
        init_wl_VIS = np.array(spec_VIS.wavelengths())[0]  # The NIR wavelengths are ~ 650-1100 nm #DOUBLE SINGLE
        # init_wl_NIR = np.array(spec_VIS.wavelengths())[0] #SINGLE
        init_wl_NIR = np.array(spec_NIR.wavelengths())[0]  # The VIS wavelengths are ~ 350-800 nm #DOUBlE
        #print("Init VIS Wavelength: {}, Init NIR Wavelength: {}".format(init_wl_VIS, init_wl_NIR))
        queue = Queue(maxsize=2)  # This queue receives the data from both the NIR and VIS spectra
        sts.runInParallel(read_VIS, read_NIR)  # DOUBLE
        # sts.runInParallel(read_VIS) #SINGLE
        if queue.full():  # If both the NIR and VIS spectra have been acquired successfully
            # print(queue.qsize()) # The queue size should be 2
            wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS = sts.assign_spectra(queue, init_wl_NIR, init_wl_VIS)  # DOUBLE
            # wavelengths_VIS, intensities_VIS, wavelengths_VIS, intensities_VIS = sts.assign_spectra(queue, init_wl_NIR, init_wl_VIS) #SINGLE
        else:
            print("Queue not full")
    except:
        print("Error while reading")
    end_time_A = time.time()
    duration = end_time_A - start_time_A
    print("Acquisition for {} seconds".format(duration))
    
    start_time_W = time.time()
    try:
        w_file_NIR(wavelengths_NIR, intensities_NIR)  # DOUBLE
        w_file_VIS(wavelengths_VIS, intensities_VIS)  # DOUBLE SINGLE
        sts.close_Spec(spec_NIR, spec_VIS)
        chain = 'r'
        #ser.write(chain.encode())
        print('Listo')
        end_time_W = time.time()
        duration = end_time_W - start_time_W
        print("Writing files for {} seconds".format(duration))
        #sb.Spectrometer.irradiance_calibration(spec_VIS, out)
    except:
        print("Error while writing files")

     # DOUBLE
    # sts.close_Spec(spec_VIS) #SINGLE

    #end_time_T = time.time()
    #duration = end_time_T - start_time_T
    #print("Total time {} seconds".format(duration))
    #__name__ = "__main__"
    # sts.close_Spec(spec_VIS)
    #sts.plot_Data(wavelengths_NIR,intensities_NIR,wavelengths_VIS,intensities_VIS)


def serial_open():

    global ser, s_read
    s_read = 1
    ser = serial.Serial(
        port='/dev/ttyUSB1',
        baudrate=57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )


if __name__ == "__main__":
    
    file_counter_NIR = 0
    file_counter_VIS = 0
    print('Reading serial')
    count_r = 0
    while 1:
        serial_open()
        print("reading command")
        # sleep(2)
        x = str(ser.read())
        while x == "b''":
            x = str(ser.read())
        global fileid
        if x == "b'n'":
            serial_open()
            print("reading file name")
            fileid = str(ser.read())
            while fileid == "b''":
                fileid = str(ser.read())
            start_time_T = time.time()
            fileid = fileid + str(datetime.datetime.now()) + str(inte) + " " + str(cap)
            file()
            print('Nuevo archivo id: ' + fileid + ' creado')
            main(inte, cap)
            ser.close()# print("listo")
        if x == "b'c'":
            serial_open()
            a = str(ser.read())
            while a == "b''":
                a = str(ser.read())
            ser.close()
            serial_open()
            b = str(ser.read())
            while b == "b''":
                b = str(ser.read())
            inte = a + b
            print(inte)
            ser.close()
            serial_open()
            cap = str(ser.read())
            while cap == "b''":
                cap = str(ser.read())
            a, b, c, d, e, f, g, h = inte
            inte = c + g
            print(inte)
            inte = int(inte)
            a, b, c, d = cap
            cap = c
            print(cap)
            serial_open()
            chain = 'r'
            ser.write(chain.encode())
            ser.close()
            cap = int(cap)
            main(inte, cap)
        if x == "b'l'":
            start_time_T = time.time()
            main(inte, cap)
            ser.close()
            # print("listo")
        if x == "b'r'":
            fileid = ("Repeat " + str(count_r) + str(datetime.datetime.now()) + str(inte) + ' ' + str(cap))
            file()
            print('Nuevo archivo id: ' + str(fileid) + ' creado')
            ser.close()
            serial_open()
            num_capS = str(ser.read())
            while num_capS == "b''":
                num_capS = str(ser.read())
            a, b, c, d = num_capS
            num_cap = int(c)
            start_time_T = time.time()
            for i in range(0, num_cap):
                start_time_m = time.time()
                main(inte, cap)
                end_time_m = time.time()
                duration = end_time_m - start_time_m
                print("Reading for {} seconds".format(duration))
                #i = i+1
            start_time_w = time.time()
            try:
                writefileV(wavelengths_VIS)
                writefileN(wavelengths_NIR)
                chain = 't'
                ser.write(chain.encode())
                ser.close()
            except:
                print("Error while writing files, try again")
            end_time_w = time.time()
            duration = end_time_w - start_time_w
            print("Time writing files {} seconds".format(duration))
            end_time_T = time.time()
            duration = end_time_T - start_time_T
            print("Total time {} seconds".format(duration))
            print("listo")
            count_r = count_r + 1
        serial_open()
        chain = 'r'
        ser.write(chain.encode())
        ser.close()
##            chain = 't'
##            ser.write(chain.encode())
##            ser.close()

            # FSM(x)
            # #print (x)
            # if x== "b'n'":
            #     chain = 'n'
            #     ser.write(chain.encode())
            #     fileid = str(ser.read())
            #     while fileid =="b''":
            #         fileid = str(ser.read())
            #     file()
            #     print('Nuevo archivo id: '+ fileid+ ' creado')

            # if x== "b'l'":
            #     chain = 'c'
            #     ser.write(chain.encode())
            #     #file()
            # if x== "b'r'":
            #     print('Espectro de referencia listo')
            #     chain = 's'
            #     ser.write(chain.encode())
            # if x == "b'c'":
            #     start_time_T = time.time()
            #     print('Comando C recibido')
            #     s_read = 2

        # main()


#wl_NIR, int_NIR, wl_VIS, int_VIS
