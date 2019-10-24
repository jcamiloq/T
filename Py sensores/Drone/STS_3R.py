# Python3 Script
# Developed by Alejandro Astudillo - April 2018
# Universidad del Valle, Cali, Colombia

# Parallel execution of the two specs: https://stackoverflow.com/questions/7207309/python-how-can-i-run-python-functions-in-parallel

from multiprocessing import Queue
import seabreeze.spectrometers as sb
import numpy as np
import time
import STS_functions as sts #This script was created in order to ease the readability of the developed script
import time
import serial
#import threading
   
def read_NIR():
    spectrum_NIR = np.array(spec_NIR.spectrum())
    queue.put(spectrum_NIR)
    
def read_VIS():
    spectrum_VIS = np.array(spec_VIS.spectrum())
    queue.put(spectrum_VIS)
    #queue.put(spectrum_VIS)

#global s_read

saveFile= open('../Data/DataNIR.txt', 'w')
saveFile.write('Inicio')
saveFile.close()

saveFile= open('../Data/DataVIS.txt', 'w')
saveFile.write('Inicio')
saveFile.close()

def w_file_NIR(arr1, arr2):
    global file_counter_NIR
    title= ('Captura número ' + str(file_counter_NIR))
    appendFile= open('../Data/DataNIR.txt', 'a')
    #appendFile.write('\n')
    appendFile.write('\n' +title+'\n')
    #appendFile.write('\n')
    saveFile.close()
    
    file_counter_NIR=file_counter_NIR+1
    
    for i in range(len(arr1)):
        appendFile=open('../Data/DataNIR.txt', 'a')
        #appendFile.write('\n')
        appendFile.write('\n' + str(arr1[i])+ " " + str(arr2[i]))
        appendFile.close()

def w_file_VIS(arr1, arr2):
    global file_counter_VIS
    title= ('Captura número ' + str(file_counter_VIS))
    appendFile= open('../Data/DataVIS.txt', 'a')
    #appendFile.write('\n')
    appendFile.write('\n' +title+ '\n')
    #appendFile.write('\n')
    saveFile.close()
    
    file_counter_VIS=file_counter_VIS+1
    
    for i in range(len(arr1)):
        appendFile=open('../Data/DataVIS.txt', 'a')
        #appendFile.write('\n')
        appendFile.write('\n'+str(arr1[i])+ " " + str(arr2[i]))
        appendFile.close()
        #print('\n')
        #print(arr1[i], " ", arr2[i],end= "")
        
def main():
    global spec_NIR, spec_VIS, wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS, queue
    spec_NIR = sts.setup_Spec(sb.Spectrometer.from_serial_number("S07678"), 100000, 5) # Serial, integration time (us), scans per avg.
    spec_VIS = sts.setup_Spec(sb.Spectrometer.from_serial_number("S09818"), 100000, 5)

    init_wl_VIS = np.array(spec_VIS.wavelengths())[0] # The NIR wavelengths are ~ 650-1100 nm
    init_wl_NIR = np.array(spec_NIR.wavelengths())[0] # The VIS wavelengths are ~ 350-800 nm
    #print("Init VIS Wavelength: {}, Init NIR Wavelength: {}".format(init_wl_VIS, init_wl_NIR))
    queue = Queue(maxsize=2) # This queue receives the data from both the NIR and VIS spectra
    
    start_time_A = time.time()
        
    sts.runInParallel(read_VIS, read_NIR)
    #sts.runInParallel(read_VIS)
    if queue.full(): # If both the NIR and VIS spectra have been acquired successfully
        #print(queue.qsize()) # The queue size should be 2
        wavelengths_NIR, intensities_NIR, wavelengths_VIS, intensities_VIS = sts.assign_spectra(queue, init_wl_NIR, init_wl_VIS)
    
    else:
        print("Queue not full")
        
    end_time_A = time.time()
    duration = end_time_A - start_time_A  
    print("Acquisition for {} seconds".format(duration))
    
    start_time_W=time.time()
    w_file_NIR(wavelengths_NIR, intensities_NIR)
    w_file_VIS(wavelengths_VIS, intensities_VIS)
    
    end_time_W = time.time()
    duration = end_time_W - start_time_W
    print("Writing files for {} seconds".format(duration))
    sts.close_Spec(spec_NIR,spec_VIS)
    chain='l'
    ser.write(chain.encode())
    print('Listo')
    end_time_T = time.time()
    duration = end_time_T - start_time_T  
    print("Total time {} seconds".format(duration))
    #sts.close_Spec(spec_VIS)
    #sts.plot_Data(wavelengths_NIR,intensities_NIR,wavelengths_VIS,intensities_VIS)
def serial_open():
    
    global ser, s_read
    s_read=1
    ser = serial.Serial(     
        port='/dev/ttyUSB0',
        baudrate = 57600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
        )
    
if __name__ == "__main__":
    file_counter_NIR=0
    file_counter_VIS=0
    
    print('Reading serial')
          
    while 1:
        serial_open()
        
        while s_read==1:
            x=str(ser.read())
            #print (x)
            if x == "b'c'":
                start_time_T = time.time()
                print('Comando C recibido')
                s_read=2
        
        main()
        


#wl_NIR, int_NIR, wl_VIS, int_VIS
