# Python3 Script
# Developed by Alejandro Astudillo - April 2018
# Universidad del Valle, Cali, Colombia

# Parallel execution of the two specs: https://stackoverflow.com/questions/7207309/python-how-can-i-run-python-functions-in-parallel

from multiprocessing import Process, Array
import seabreeze.spectrometers as sb
import numpy as np
import matplotlib.pyplot as plt
import time

def setup_Spec(spectrometer, int_time, scans_to_average):
    
    spectrometer.integration_time_micros(int_time)
    spectrometer.scans_to_average(scans_to_average)
    
    return spectrometer
    
def close_Spec():
    spec_NIR.close()
    spec_VIS.close()

def plot_Data():
    #global wavelenghts_NIR,intensities_NIR,wavelenghts_VIS,intensities_VIS
    plt.subplot(3,1,1)
    plt.title("STS Spectrum") 
    plt.xlabel("Wavelenghts [um]") 
    plt.ylabel("NIR Intensities") 
    plt.plot(wavelenghts_NIR,intensities_NIR) 
    plt.subplot(3,1,3)
    plt.xlabel("Wavelenghts [um]") 
    plt.ylabel("VIS Intensities") 
    plt.plot(wavelenghts_VIS,intensities_VIS) 
    plt.show()
    
def read_NIR():
    global wavelenghts_NIR, intensities_NIR
    spectrum_NIR = np.array(spec_NIR.spectrum())
    #wavelenghts_NIR = np.array(spec_NIR.wavelengths())
    #intensities_NIR = np.array(spec_NIR.intensities())
    wavelenghts_NIR = spectrum_NIR[0,:]
    intensities_NIR = spectrum_NIR[1,:]    

def read_VIS():
    global wavelenghts_VIS, intensities_VIS
    spectrum_VIS = np.array(spec_VIS.spectrum())
    wavelenghts_VIS = spectrum_VIS[0,:]
    intensities_VIS = spectrum_VIS[1,:]
    
def runInParallel(*fns):
    proc = []
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()
    
def main():
    global spec_NIR, spec_VIS, wavelenghts_NIR, intensities_NIR, wavelenghts_VIS, intensities_VIS
    #devices = sb.list_devices()
    #spec_NIR = sb.Spectrometer(devices[0])
    #spec_NIR = setup_Spec(sb.Spectrometer.from_serial_number("S09067"), 100000, 5)
    #spec_VIS = setup_Spec(sb.Spectrometer.from_serial_number("S09818"), 100000, 5)
    #intensities_VIS = np.arange(1024)
    #wavelenghts_VIS = np.arange(1024)
    #intensities_NIR = np.arange(1024)
    #wavelenghts_NIR = np.arange(1024)
    
    start_time = time.time()
    
    runInParallel(read_VIS, read_NIR)

    end_time = time.time()
    duration = end_time - start_time  
    
    #print(arr[:])
    print("I slept for {} seconds".format(duration))
    print (np.max(intensities_NIR))
    print (np.max(intensities_VIS))
    
    #plot_Data()
    close_Spec()

global spec_NIR, spec_VIS, wavelenghts_NIR, intensities_NIR, wavelenghts_VIS, intensities_VIS
if __name__ == "__main__":
    spec_NIR = setup_Spec(sb.Spectrometer.from_serial_number("S09067"), 100000, 5)
    spec_VIS = setup_Spec(sb.Spectrometer.from_serial_number("S09818"), 100000, 5)
    intensities_VIS = np.zeros((1024,1))
    wavelenghts_VIS = np.zeros((1024,1))
    intensities_NIR = np.zeros((1024,1))
    wavelenghts_NIR = np.zeros((1024,1))
    
    main()

