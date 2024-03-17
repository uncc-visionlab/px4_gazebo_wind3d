import os, sys
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

channels = ['u_00000001', 'u_00000002', 'u_00000003', 'u_00000004', 'u_00000005']
timestep = 0.01
nb_timesteps = 968

original_df = pd.read_csv('original_U_velocity.csv', index_col=False)
fft_df = pd.read_csv('FFT_U_velocity.csv', index_col=False)

plt.figure(0, figsize=(18, 9))
for ichannel, channel in enumerate(channels):
	if channel not in original_df: sys.exit("ERROR: channel not found in original data: " + channel)
	if channel + '_FFT-Frequency' not in fft_df: sys.exit("ERROR: column not found: " + channel + '_FFT-Frequency')
	if channel + '_FFT-REAL' not in fft_df: sys.exit("ERROR: column not found: " + channel + '_FFT-REAL')
	if channel + '_FFT-IMAG' not in fft_df: sys.exit("ERROR: column not found: " + channel + '_FFT-IMAG')
	
	xfrq = fft_df[channel + '_FFT-Frequency']
	xfft = fft_df[channel + '_FFT-REAL'] + 1j * fft_df[channel + '_FFT-IMAG']
	
	ifft_data = []
	for n, this_time in enumerate(original_df['time']):
	    this_ifft = 0j
	    for k in range(len(xfft)):
	        if xfrq[k] == 0.0:
	            this_ifft += xfft[k]
	        else:
	            this_ifft += xfft[k] * np.exp(1j*2*np.pi*(this_time-original_df['time'][0])*xfrq[k])
	            this_ifft += np.conjugate(xfft[k]) * np.exp(-1j*2*np.pi*(this_time-original_df['time'][0])*xfrq[k])
	    this_ifft /= nb_timesteps
	    ifft_data.append(np.real(this_ifft))
	
	plt.subplot(len(channels), 1, ichannel+1)
	plt.plot(original_df['time'], original_df[channel], label='Original data')
	plt.plot(original_df['time'], ifft_data, '-.', label='Reconstructed data')
	
	plt.xlabel('Time [s]')
	plt.ylabel(channel)
	plt.legend(loc='upper right')
	plt.grid(True,linestyle='-')
plt.tight_layout()
plt.show()


