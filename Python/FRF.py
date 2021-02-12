#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pytta

if __name__ == '__main__':
          #pytta.list_devices()
          #device = pytta.get_device_from_user()
          
          #signalOUT = pytta.generate.random_noise(kind='white') 
          signalOUT = pytta.generate.sweep(fftDegree=18)  # Generate sine sweep SignalObj
          signalIN = np.zeros((2, signalOUT.numSamples))
          
          for Nchan in range(1, 3):     
                
                FRFparams = {
                    'excitation'  : signalOUT,        # Passes the excitation signal
                    'samplingRate': 44100,            # Frequency of sampling
                    'freqMin'     : 100,              # Minimum frequency of interest NOT WORKING
                    'freqMax'     : 20000,            # Maximum frequency of interest NOT WORKING
                    'device'      : 4,                # Device number provided at runtime
                    'inChannels'  : [Nchan],          # List of hardware channels to be used
                    'outChannels' : [Nchan],          # List of hardware channels to be used
                    'comment'     : 'Testing; 1, 2.'  # just a comentary
                }

                # Generates the configuration for an impulse response measurement
                FRFmeasurement = pytta.generate.measurement('frf', **FRFparams)                
                frf = FRFmeasurement.run()

                #Plot
                frf.plot_time(decimalSep='.')
                frf.plot_freq(decimalSep='.')
           
          #%% Save the measurement
          #path = 'data\\'
          #name = 'myir'
          #filename = f'{path}{name}'
      
          # Save PyTTaObj as HDF5 file
          #pytta.save(filename, rec)
      
          # Export wave file of SignalObj
          # pytta.write_wav(filename+'.wav', rec.IR)
