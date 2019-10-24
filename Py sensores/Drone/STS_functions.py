import seabreeze.spectrometers as sb
from multiprocessing import Process
from multiprocessing import Queue
import numpy as np
import matplotlib.pyplot as plt
#import time

global x_nir, y_nir, x_vis, y_vis

def setup_Spec(spectrometer, int_time, scans_to_average):
    #devices = sb.list_devices()
    #spec_NIR = sb.Spectrometer(devices[0])
    #spec_NIR = sb.Spectrometer.from_serial_number("S09067")
    spectrometer.integration_time_micros(int_time)
    spectrometer.scans_to_average(scans_to_average)
    return spectrometer
    
def close_Spec(*specs):
    for sp in specs:
        sp.close()
        print("{} closed".format(sp))
        
def runInParallel(*fns):
    proc = []
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()
    
def get_all_queue_result(queue):
    result_list = []
    while not queue.empty():
        result_list.append(queue.get())
    return result_list

def assign_spectra(q, minWL_NIR, minWL_VIS):
    #res = get_all_queue_result(q)
    #spectrum_1 = res[0]
    #spectrum_2 = res[1]
    spectrum_1 = q.get() # Get data from the queue
    spectrum_2 = q.get()
    
    if abs(spectrum_1[0,0]-minWL_VIS)<0.000000001 and abs(spectrum_2[0,0]-minWL_NIR)<0.000000001:
        wl_VIS = spectrum_1[0,:]
        inten_VIS = spectrum_1[1,:]
        wl_NIR = spectrum_2[0,:]
        inten_NIR = spectrum_2[1,:]
    elif abs(spectrum_1[0,0]-minWL_NIR)<0.000000001 and abs(spectrum_2[0,0]-minWL_VIS)<0.000000001:
        wl_VIS = spectrum_2[0,:]
        inten_VIS = spectrum_2[1,:]
        wl_NIR = spectrum_1[0,:]
        inten_NIR = spectrum_1[1,:]
    else:
        print("Not suitable data")
    x_vis=wl_VIS
    y_vis=inten_VIS
    x_nir=wl_NIR
    y_nir=inten_NIR
    return wl_NIR,inten_NIR,wl_VIS,inten_VIS


def plot_Data(wl_NIR, int_NIR, wl_VIS, int_VIS):
    plt.subplot(3,1,1)
    plt.title("STS Spectrum") 
    plt.xlabel("Wavelengths [um]") 
    plt.ylabel("NIR Intensities") 
    plt.plot(wl_NIR,int_NIR) 
    plt.subplot(3,1,3)
    plt.xlabel("Wavelengths [um]") 
    plt.ylabel("VIS Intensities") 
    plt.plot(wl_VIS,int_VIS) 
    plt.show()
def ver():
    #arr=np.array([1, 2, 3, 4, 5])
    for i in range(len(wl_NIR)):
        print(wl_NIR[i], " ", end= "")