#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import ADS1256
import RPi.GPIO as GPIO
import numpy as np
from functools import reduce
import math
import matplotlib.pyplot as plt

# Constants and configurations
SET_CHANNEL = True

FREQUENCY = 30000  # Target ADC sampling frequency (in Hz)
GPIO_FREQ_FACTOR = 2  # Ratio between GPIO and ADC frequencies


class FrequencyDetector:
    def __init__(self):
        self.current_time = time.time()
        self.last_time = self.current_time
        self.start_time = self.current_time
        self.count = 0
        
    def update(self):
        self.count += 1
        self.current_time = time.time()
        
    def realtime_update(self):
        self.last_time = self.current_time
        self.current_time = time.time()
        self.count += 1
       
        dt = self.current_time - self.last_time
        if dt > 0:
            freq = 1/dt
            sys.stdout.write("\r")
            sys.stdout.write(f"{freq:.4f} Hz       ")
            sys.stdout.flush()
    def get_freq(self):
        return self.count / (self.current_time - self.start_time)
    
class FrequencyRegulator:
    def __init__(self, frequency):
        self.frequency = frequency
        self.start_time = time.time()
        self.dt = 1/self.frequency
        self.next_pulse_time = self.start_time + self.dt
    
    def is_next_pulse_ok(self):
        if time.time() > self.next_pulse_time:
            self.next_pulse_time += self.dt
            return True
        return False
        

# PRBS Generator function with adjustable length and taps
def generate_prbs(length, taps):
    bits = int(math.log2(length + 1))
    tap_binary = bin(taps)[2:].zfill(bits)[::-1]
    tap_binary = np.array([int(b) for b in tap_binary])
    register = np.array([1] + [0] * (bits - 1))  # Initial register state
    
    output = []
    for _ in range(length):
        feedback = reduce(lambda x, y: x ^ y, register & tap_binary)
        output.append(register[-1])
        register = np.insert(register, 0, feedback)[:bits]
    
    return output


# PRBS configuration
PRBS_LENGTH = 65535   # Adjustable PRBS sequence length
TAPS = 0xD008  # Example taps for generating PRBS

PRBS_PINS = [7, 12, 16, 20, 21]  # Example GPIO pins, representing 5 channels

# Generate a single PRBS sequence
prbs_sequence = generate_prbs(PRBS_LENGTH, TAPS)

# Define phase shifts for each GPIO pin
phase_shifts = [0, PRBS_LENGTH // 5, (2 * PRBS_LENGTH) // 5, (3 * PRBS_LENGTH) // 5, (4 * PRBS_LENGTH) // 5]


# GPIO setup
GPIO.setmode(GPIO.BCM)    
GPIO.setwarnings(False)
GPIO.setup(7, GPIO.OUT)
GPIO.setup(12, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

# ADC setup
ADC = ADS1256.ADS1256()
ADC.ADS1256_init()
ADC.ADS1256_WriteReg(0x00,0x02)


freq_detector = FrequencyDetector()
freq_regulator = FrequencyRegulator(FREQUENCY)

gpio_freq_factor = GPIO_FREQ_FACTOR


adc_data = []
time_data = []


gpio_count = 0

    
start_time = time.time()

count = 0 

while(len(adc_data) < 500):
    if not freq_regulator.is_next_pulse_ok():
        continue
    
    freq_detector.update()
    
    if gpio_count == gpio_freq_factor:
        gpio_count = 0
        for i in range(7):
            for idx in range(len(PRBS_PINS)):
                GPIO.output(PRBS_PINS[idx], GPIO.HIGH if prbs_sequence[(phase_shifts[idx] + count) % PRBS_LENGTH] == 1 else GPIO.LOW) 
                if SET_CHANNEL:   
                    ADC.ADS1256_SetChannal(i)
                    ADC.ADS1256_WriteCmd(0xFC) 
                    ADC.ADS1256_WriteCmd(0x00) 
                    adc_value = ADC.ADS1256_Read_ADC_Data()*5.0/0x7fffff

        count += 1

    gpio_count += 1
    

    adc_data.append(adc_value)
    time_data.append(time.time() - start_time)
print(f"Frequency: {freq_detector.get_freq():.4f} Hz")

# plot the data
# plt.figure(figsize=(10, 5))
# plt.plot(time_data[400:500], adc_data[400:500])
# plt.ylim(-1,6)
# plt.xlabel("Time (s)")
# plt.ylabel("ADC Value")
# plt.title("ADC Value vs Time")
# plt.grid(True)
# plt.savefig("adc_value_vs_time.png")


