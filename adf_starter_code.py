#!/usr/bin/env python3

import spidev
import RPi.GPIO as GPIO
import time
import math
import sys
import argparse
from decimal import Decimal, getcontext

# Constants from ADF4351.h
ADF_FREQ_MAX = 4294967295
ADF_FREQ_MIN = 34385000
ADF_PFD_MAX = 32000000.0
ADF_PFD_MIN = 125000.0
ADF_REFIN_MAX = 250000000
REF_FREQ_DEFAULT = 100000000

# Pin definitions
CE = 2    # Chip Enable
LD = 25   # Lock Detect

# SPI settings
SPI_BUS = 0
SPI_DEVICE = 0
SPI_SPEED = 10000000  # 10 MHz
SPI_MODE = 0b00

# Frequency steps allowed (in Hz)
FREQ_STEPS = [1000, 5000, 10000, 50000, 100000, 500000, 1000000]

class ADF4351:
    def __init__(self, ref_freq=REF_FREQ_DEFAULT):
        self.ref_freq = ref_freq
        self.enabled = False
        self.cfreq = 0
        self.chan_step = FREQ_STEPS[0]
        self.rd2refdouble = 0
        self.rcounter = 25
        self.rd1rdiv2 = 0
        self.band_sel_clock = 80
        self.clk_div = 150
        self.prescaler = 0
        self.pwr_level = 0
        self.registers = [0] * 6
        getcontext().prec = 10  # Set decimal precision

        # Initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(SPI_BUS, SPI_DEVICE)
        self.spi.max_speed_hz = SPI_SPEED
        self.spi.mode = SPI_MODE
        self.spi.bits_per_word = 8

    def init_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CE, GPIO.OUT)
        GPIO.setup(LD, GPIO.IN)
        GPIO.output(CE, GPIO.LOW)

    def write_register(self, reg_value):
        bytes_to_write = [
            (reg_value >> 24) & 0xFF,
            (reg_value >> 16) & 0xFF,
            (reg_value >> 8) & 0xFF,
            reg_value & 0xFF
        ]
        print(f"\nWriting register value: 0x{reg_value:08X}")
        print(f"Bytes to write: [0x{bytes_to_write[0]:02X}, 0x{bytes_to_write[1]:02X}, 0x{bytes_to_write[2]:02X}, 0x{bytes_to_write[3]:02X}]")
        self.spi.xfer2(bytes_to_write)
        time.sleep(0.0001)

    def gcd(self, a, b):
        while b:
            a, b = b, a % b
        return a

    def calculate_registers(self, freq_hz):
        print(f"\nCalculating registers for {freq_hz/1e6:.3f} MHz")
        print(f"Reference frequency: {self.ref_freq/1e6:.3f} MHz")

        if not (ADF_FREQ_MIN <= freq_hz <= ADF_FREQ_MAX):
            raise ValueError(f"Frequency must be between {ADF_FREQ_MIN/1e6:.3f} MHz and {ADF_FREQ_MAX/1e6:.3f} MHz")

        # Calculate RF divider
        localosc_ratio = 2200000000 // freq_hz
        rf_div = 1
        rf_div_sel = 0
        
        while rf_div <= localosc_ratio and rf_div <= 64:
            rf_div *= 2
            rf_div_sel += 1

        vco_freq = freq_hz * rf_div
        print(f"VCO frequency: {vco_freq/1e6:.3f} MHz")
        print(f"RF divider: {rf_div} (selector: {rf_div_sel})")
        
        # Set prescaler based on VCO frequency
        self.prescaler = 1 if vco_freq > 3600000000 else 0
        print(f"Prescaler: {self.prescaler}")

        # Calculate PFD frequency
        pfd_freq = Decimal(self.ref_freq) * (Decimal(1 + self.rd2refdouble) / 
                                        Decimal(self.rcounter * (1 + self.rd1rdiv2)))
        print(f"PFD frequency: {float(pfd_freq)/1e6:.3f} MHz")

        # Calculate N divider values
        N = Decimal(vco_freq) / pfd_freq
        N_int = int(N)
        print(f"N = {float(N):.3f} (INT: {N_int})")
        
        if self.prescaler == 0 and not (23 <= N_int <= 65535):
            raise ValueError(f"N_int {N_int} out of range for prescaler=0")
        elif self.prescaler == 1 and not (75 <= N_int <= 65535):
            raise ValueError(f"N_int {N_int} out of range for prescaler=1")

        # Calculate fractional values
        MOD = int(pfd_freq / self.chan_step)
        MOD = min(MOD, 4095)  # Limit to 12 bits
        print(f"Initial MOD: {MOD}")
                
        FRAC = int((N - N_int) * MOD + Decimal('0.5'))
        print(f"Initial FRAC: {FRAC}")
        
        # Optimize FRAC and MOD
        if FRAC != 0:
            gcd = self.gcd(FRAC, MOD)
            if gcd > 1:
                FRAC //= gcd
                MOD //= gcd
                print(f"Optimized FRAC/MOD: {FRAC}/{MOD} (GCD: {gcd})")

        if FRAC >= MOD:
            raise ValueError(f"FRAC ({FRAC}) must be less than MOD ({MOD})")

        print("\nCalculating register values:")
        # Build registers
        r0 = (N_int << 15) | (FRAC << 3) | 0
        print(f"R0 = 0x{r0:08X} (N_int:{N_int}, FRAC:{FRAC})")
        
        r1 = (MOD << 3) | (self.prescaler << 27) | 1
        print(f"R1 = 0x{r1:08X} (MOD:{MOD}, Prescaler:{self.prescaler})")
        
        # R2: Enhanced settings for noise and spurs
        r2 = (0x7 << 9)  # Charge pump current
        if FRAC == 0:    # Integer-N mode
            r2 |= (1 << 7) | (1 << 8)  # LDP and LDF settings
            print("Integer-N mode enabled")
        r2 |= (1 << 6) | 2  # PD polarity and control bits
        r2 |= (self.rcounter << 14)
        print(f"R2 = 0x{r2:08X}")

        
        # R3: Band select and auxiliary settings
        r3 = (1 << 23)   # Band select clock mode
        if FRAC == 0:
            r3 |= (1 << 21) | (1 << 22)  # Integer-N optimizations
        r3 |= 3
        print(f"R3 = 0x{r3:08X}")
        
        # R4: Output and divider settings
        r4 = (self.pwr_level << 3) | (1 << 5) | (self.band_sel_clock << 12) | (rf_div_sel << 20) | (1 << 23) | 4
        print(f"R4 = 0x{r4:08X} (Power:{self.pwr_level}, RF_Div:{rf_div_sel})")
        
        # R5: Lock detect and misc settings
        r5 = (1 << 22) | 5
        print(f"R5 = 0x{r5:08X}")

        self.registers = [r0, r1, r2, r3, r4, r5]
        return True

    def set_frequency(self, freq_hz):
        print(f"\nSetting frequency to {freq_hz/1e6:.3f} MHz")
        if self.calculate_registers(freq_hz):
            # self.registers contains [r5, r4, r3, r2, r1, r0]
            print("\nWriting registers in order (5 to 0):")
            for i in range(6):
                print(f"\nWriting R{i}")
                self.write_register(self.registers[i])
                time.sleep(0.001)
            return True
        return False

    def set_power_level(self, level):
        if not 0 <= level <= 3:
            raise ValueError("Power level must be between 0 and 3")
        self.pwr_level = level
        
    def enable(self):
        self.enabled = True
        GPIO.output(CE, GPIO.HIGH)

    def disable(self):
        self.enabled = False
        GPIO.output(CE, GPIO.LOW)

    def cleanup(self):
        self.spi.close()
        GPIO.cleanup()

def main():
    parser = argparse.ArgumentParser(description='ADF4351 Frequency Synthesizer Control')
    parser.add_argument('frequency', type=float, help='Frequency in MHz (35-4400)')
    parser.add_argument('--power', type=int, choices=[0,1,2,3], default=3,
                      help='Output power level (0=-4dBm, 1=-1dBm, 2=2dBm, 3=5dBm)')
    parser.add_argument('--ref', type=float, default=25.0,
                      help='Reference frequency in MHz (default: 25.0)')
    parser.add_argument('--step', type=float, choices=[s/1e6 for s in FREQ_STEPS],
                      default=0.001, help='Frequency step size in MHz')

    args = parser.parse_args()

    # Convert MHz to Hz
    freq_hz = int(args.frequency * 1e6)
    ref_freq = int(args.ref * 1e6)
    
    try:
        synth = ADF4351(ref_freq=ref_freq)
        synth.init_gpio()
        synth.set_power_level(args.power)
        synth.chan_step = int(args.step * 1e6)
        
        print(f"Setting frequency to {args.frequency} MHz...")
        if synth.set_frequency(freq_hz):
            print("Frequency set successfully")
            synth.enable()
            
            print("\nPress Ctrl+C to exit...")
            while True:
                time.sleep(1)
                
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'synth' in locals():
            synth.disable()
            synth.cleanup()

if __name__ == "__main__":
    main()