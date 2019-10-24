from multiprocessing import Queue
import seabreeze.spectrometers as sb
import numpy as np
import time
from time import sleep
import STS_functions as sts  # This script was created in order to ease the readability of the developed script
import time
import serial
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--connect", dest="conect", help="Número de captura")
results = parser.parse_args()
datanum = results.conect
print(str(datanum))

saveFile = open('/home/pi/Desktop/Data/DataNIR%s.txt' % (datanum), 'w')
saveFile.write('Inicio')
saveFile.close()

saveFile = open('/home/pi/Desktop/Data/DataVIS%s.txt' % (datanum), 'w')
saveFile.write('Inicio')
saveFile.close()

estado = 'i'

# Estados


def EDOi(entrada):
    global estado
    print('Estado Inicial')
    sleep(2)
    if entrada == "b'n'":
        print("Nombre de nuevo archivo?")
        datanum = input()
        chain = str(datanum)
        ser.write(chain.encode())
        sleep(2)
        #start_time_T = time.time()
        print('Nuevo archivo fileid: ' + datanum + ' creado')
        while x != "b'r'":
            x = str(ser.read())
        print('Listo recibido')
        main()
        #chain = "b'l'"
        #ser.write(chain.encode())
        #estado = 0
        print('Trancisión hacia 0')
    else:
        estado = 'i'

def EDO0(entrada):
    global estado
    print('Estado 0')
    sleep(2)
    if entrada == "b'l'":
        #start_time_T = time.time()
        #print('Comando C recibido')
        #s_read = 2
        while x != "b'r'":
            x = str(ser.read())
        main()
        estado = 0
    else:
        print('Trancisión hacia i')
        estado = "i"
        

# Finite State Machine (FSM)

def FSM(entrada):
    global estado
    switch = {
        'i': EDOi,
        0: EDO0,
        #1: EDO1,
        #2: EDO2,
    }
    func = switch.get(estado, lambda: None)
    return func(entrada)


def serial_open():

    global ser, s_read
    s_read = 1
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )


def read_NIR():
    spectrum_NIR = np.array(spec_NIR.spectrum())
    queue.put(spectrum_NIR)


def read_VIS():
    spectrum_VIS = np.array(spec_VIS.spectrum())
    queue.put(spectrum_VIS)
    # queue.put(spectrum_VIS) #SINGLE


def w_file_NIR(arr1, arr2):
    global file_counter_NIR
    title = ('Captura número ' + str(file_counter_NIR))
    appendFile = open('/home/pi/Desktop/Data/DataNIR%s.txt' % (datanum), 'a')
    # appendFile.write('\n')
    appendFile.write('\n' + title + '\n')
    # appendFile.write('\n')
    saveFile.close()

    file_counter_NIR = file_counter_NIR + 1

    for i in range(len(arr1)):
        appendFile = open('/home/pi/Desktop/Data/DataNIR%s.txt' % (datanum), 'a')
        # appendFile.write('\n')
        appendFile.write('\n' + str(arr1[i]) + " " + str(arr2[i]))
        appendFile.close()


def w_file_VIS(arr1, arr2):
    global file_counter_VIS
    title = ('Captura número ' + str(file_counter_VIS))
    appendFile = open('/home/pi/Desktop/Data/DataVIS%s.txt' % (datanum), 'a')
    # appendFile.write('\n')
    appendFile.write('\n' + title + '\n')
    # appendFile.write('\n')
    saveFile.close()

    file_counter_VIS = file_counter_VIS + 1

    for i in range(len(arr1)):
        appendFile = open('/home/pi/Desktop/Data/DataVIS%s.txt' % (datanum), 'a')
        # appendFile.write('\n')
        appendFile.write('\n' + str(arr1[i]) + " " + str(arr2[i]))
        appendFile.close()


def main():
    s_read = 1
    #chain = 'r'
    time.sleep(4)
    #ser.write(chain.encode())
    print('Listo')
# def main():
##    global spec_NIR, spec_VIS, wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS, queue, out
# spec_NIR = sts.setup_Spec(sb.Spectrometer.from_serial_number("S07678"), 100000, 5)  # Serial, integration time (us), scans per avg. #DOUBLE
# spec_VIS = sts.setup_Spec(sb.Spectrometer.from_serial_number("S07678"), 100000, 5)  # DOUBLE #SINGLE
# out = []
# init_wl_VIS = np.array(spec_VIS.wavelengths())[0]  # The NIR wavelengths are ~ 650-1100 nm  #SINGLE      #DOUBLE
# init_wl_NIR = np.array(spec_NIR.wavelengths())[0]  # The VIS wavelengths are ~ 350-800 nm             #DOUBLE
# init_wl_NIR = np.array(spec_VIS.wavelengths())[0]  # SINGLE
# print("Init VIS Wavelength: {}, Init NIR Wavelength: {}".format(init_wl_VIS, init_wl_NIR))
# queue = Queue(maxsize=2)  # This queue receives the data from both the NIR and VIS spectra
##
##    start_time_A = time.time()
##
# sts.runInParallel(read_VIS, read_NIR)                                                             #DOUBLE
# sts.runInParallel(read_VIS)  # SINGLE
# sts.runInParallel(read_VIS)
# if queue.full():  # If both the NIR and VIS spectra have been acquired successfully
# print(queue.qsize()) # The queue size should be 2
# wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS = sts.assign_spectra(queue, init_wl_NIR, init_wl_VIS) #DOUBLE
# wavelengths_VIS, intensities_VIS, wavelengths_VIS, intensities_VIS = sts.assign_spectra(queue, init_wl_NIR, init_wl_VIS)  # SINGLE
##
# else:
##        print("Queue not full")
##
##    end_time_A = time.time()
##    duration = end_time_A - start_time_A
##    print("Acquisition for {} seconds".format(duration))
# s_read=1
##    start_time_W = time.time()
# w_file_NIR(wavelengths_NIR, intensities_NIR)                                                      #DOUBLE
# w_file_VIS(wavelengths_VIS, intensities_VIS)  # DOUBLE #SINGLE
##
# sb.Spectrometer.irradiance_calibration(spec_VIS, out)
##
##    end_time_W = time.time()
##    duration = end_time_W - start_time_W
##    print("Writing files for {} seconds".format(duration))
# sts.close_Spec(spec_NIR, spec_VIS) #DOUBLE
# sts.close_Spec(spec_VIS) #SINGLE
##    chain = 'r'
# ser.write(chain.encode())
# print('Listo')
##    end_time_T = time.time()
##    duration = end_time_T - start_time_T
##    print("Total time {} seconds".format(duration))
# sts.close_Spec(spec_VIS)
# sts.plot_Data(wavelengths_NIR,intensities_NIR,wavelengths_VIS,intensities_VIS)


def serial_open():

    global ser, s_read
    s_read = 1
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=38400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
    )


if __name__ == "__main__":
    file_counter_NIR = 0
    file_counter_VIS = 0
    print('Reading serial')
    counter = ""
    ready = ""
    while 1:
        serial_open()
        #sleep(2)
        print("¿Qué comando desea ingresar?")
        counter = input()
        print(counter)
        ser.write(counter.encode())
        while s_read == 1:
            if counter == 'n':
                print("Nombre de nuevo archivo?")
                datanum = input()
                chain = str(datanum)
                ser.write(chain.encode())
                #sleep(2)
                start_time_T = time.time()
                print('Nuevo archivo fileid: ' + datanum + ' creado')
                main()
                print("LISTO")
                x = str(ser.read())
                while x != "b'r'":
                    x = str(ser.read())
                print('Listo recibido')
                end_time_T = time.time()
                duration = end_time_T - start_time_T
                print("Total time {} seconds".format(duration))
                s_read=2
            if counter == 'l':
                start_time_T = time.time()
                print('Comando l recibido')
                sleep(0.3)
                main()
                x = str(ser.read())
                while x != "b'r'":
                    x = str(ser.read())
                end_time_T = time.time()
                duration = end_time_T - start_time_T
                print("Total time {} seconds".format(duration))
                s_read = 2
            if counter == 'r':
                print("Cuantas capturas desea?")
                num_cap = input()
                sleep(0.3)
                chain = str(num_cap)
                ser.write(chain.encode())
                datanum = "repeat"
                #sleep(2)
                start_time_T = time.time()
                print('Nuevo archivo fileid: ' + datanum + ' creado' + ' nùmero de capturas: ' + num_cap)
                main()
                print("LISTO")
                x = str(ser.read())
                while x != "b't'":
                    x = str(ser.read())
                print('Listo recibido')
                end_time_T = time.time()
                duration = end_time_T - start_time_T
                print("Total time {} seconds".format(duration))
                s_read=2
            if counter == "c":
                print("Ingrese tiempo de integración seguido del número de capturas")
                text = input()
                text = text.split(",")
                inte = text[0]
                cap = text[1]
                chain = inte
                ser.write(chain.encode())
                sleep(1)
                chain = cap
                ser.write(chain.encode())
                sleep(1)
                x = str(ser.read())
                while x != "b'r'":
                    x = str(ser.read())
                print("parametros de calibración recibidos")
                s_read=2
            #x = str(ser.read(
        
            #FSM(counter)
        #     #print (x)
        #     if x == "b'n'":
        #         chain = str(datanum)
        #         ser.write(chain.encode())
        #         #start_time_T = time.time()
        #         print('Nuevo archivo fileid: ' + datanum + ' creado')
        #         chain= "b'l'"
        #         ser.write(chain.encode())
        #     if x == "b'c'":
        #         start_time_T = time.time()
        #         print('Comando C recibido')
        #         #s_read = 2
        #         main()
        #     if x == "b's'":
        #         print('llego S')
        #         chain = 'c'
        #         ser.write(chain.encode())
        #         while x != "b'l'":
        #             x = str(ser.read())
        #         if x == "b'l'":
        #             print("Listo recibido")
        #             s_read=2
        #     if x == "b''":
        #         x = str(ser.read())


# while 1:
#    if ready== b'l':
#        print("¿Qué comando desea enviar?")
#        counter=input()
#        ser.write(counter.encode())
#        ready=ser.read()
#        #print(ready)
#    else:
#        ready=ser.read()
#        #print(ready)
#
#    #time.sleep(1)
