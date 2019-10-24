# Python3 Script
# Developed by Alejandro Astudillo - April 2018
# Universidad del Valle, Cali, Colombia

# Parallel execution of the two specs: https://stackoverflow.com/questions/7207309/python-how-can-i-run-python-functions-in-parallel

import multiprocessing as mp
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
    spectrum_NIR = np.array(spec_NIR.spectrum())
    #wavelenghts_NIR = np.array(spec_NIR.wavelengths())
    #intensities_NIR = np.array(spec_NIR.intensities())
    wavelenghts_NIR = spectrum_NIR[0,:]
    intensities_NIR = spectrum_NIR[1,:]
    return intensities_NIR

def read_VIS():
    #global spectrum_VIS, wavelenghts_VIS, intensities_VIS
    spectrum_VIS = np.array(spec_VIS.spectrum())
    wavelenghts_VIS = spectrum_VIS[0,:]
    intensities_VIS = spectrum_VIS[1,:]
    return intensities_VIS

def runInParallel(*fns):
    results = {}
    pool = mp.Pool()    
    for func in fns:
        def callback(result, func=func):
            results[func] = result
        pool.apply_async(func, callback=callback)
    pool.close()
    pool.join()
    return results

def main():
    global spec_NIR, spec_VIS, wavelenghts_NIR, intensities_NIR, wavelenghts_VIS, intensities_VIS
    #devices = sb.list_devices()
    #spec_NIR = sb.Spectrometer(devices[0])
    spec_NIR = setup_Spec(sb.Spectrometer.from_serial_number("S09067"), 100000, 5)
    spec_VIS = setup_Spec(sb.Spectrometer.from_serial_number("S09818"), 100000, 5)
    #pool = mp.Pool()
    
    start_time = time.time()
    #wavelenghts_VIS, intensities_VIS = pool.apply_async(read_VIS)    
    #wavelenghts_NIR, intensities_NIR = pool.apply_async(read_NIR)
    results = runInParallel(read_VIS, read_NIR)
    #answer1 = result1.get(timeout=10)
    #anwer2 = result2.get(timeout=10)
    #print (np.array(results))
    #output = [p.get() for p in results
    
    end_time = time.time()
    duration = end_time - start_time  
    
    print(results)
    print("I slept for {} seconds".format(duration))
    #print (np.max(intensities_NIR))
    #print (np.max(intensities_VIS))
    
    #plot_Data()
    close_Spec()

if __name__ == "__main__":
    main()


