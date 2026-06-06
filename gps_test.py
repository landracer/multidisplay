#!/usr/bin/env python3
"""
Quick script to capture and log MD serial data for 30 seconds
to check for GPS status information.
"""

import serial
import time
import sys

def main():
    # Try to connect to the MD unit on ACM6
    try:
        ser = serial.Serial('/dev/ttyACM6', 115200, timeout=1)
        print("Connected to MD unit on /dev/ttyACM6")
        print("Capturing data for 30 seconds...")
        
        # Open log file
        with open('md_serial_log.txt', 'w') as f:
            start_time = time.time()
            while time.time() - start_time < 30:
                if ser.in_waiting > 0:
                    data = ser.readline()
                    if data:
                        line = data.decode('utf-8', errors='ignore').strip()
                        if line:
                            print(line)
                            f.write(line + '\n')
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
                
        ser.close()
        print("Logging complete. Check md_serial_log.txt for GPS information.")
        
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        print("Make sure the MD unit is connected and the port is correct.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("Logging interrupted by user.")
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    main()