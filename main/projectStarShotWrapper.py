import struct
import time
import numpy as np
import serial

# -------------------Configuration-----------------------------------
CFG_FILE = "area_scanner_68xx_ODS.cfg"
COM_PORTS = {"cfg": "COM6", "data": "COM4"}
BAUDRATE = 921600
#---------------------End of Configuration---------------------------


#-----------------Magic Word---------------------------
MAGIC_WORD = [2, 1, 4, 3, 6, 5, 8, 7]
#----------------End of magic word---------------------


#-------------------------Parsing Logic---------------
def parse_cfg_file(file_path):
    """Parses the radar configuration file into a list of commands."""
    commands = []
    try:
        with open(file_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('%'):  # Skip comments
                    commands.append(line)
    except FileNotFoundError:
        print(f"Error: Configuration file {file_path} not found.")
    return commands


def send_config(port, commands):
    """Sends radar configuration commands via serial port.

    Runs every time, with the configurations specified in CFG_FILE
    
    """
    if port:
        for cmd in commands:
            port.write((cmd + '\n').encode())
            time.sleep(0.1)
            print(f"Sent: {cmd}")


#---------------------End of parsing logic-------------------





#-------------------Serial port init-------------------------
def init_serial_port(port, baudrate=BAUDRATE):
    """Initializes a serial port for communication."""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Opened port {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening port {port}: {e}")
        return None
#-------------------End of Serial port init-----------------------


#TODO: implement packet parsing logic






if __name__ == "__main__":
    
    #Pass CFG commands to IWR6843ISK-ODS 
    cli_commands = parse_cfg_file(CFG_FILE) 
    if not cli_commands:
        print("No valid configuration commands found. Exiting...")
        exit()

    cfg_port = init_serial_port(COM_PORTS["cfg"])
    data_port = init_serial_port(COM_PORTS["data"])

    if not cfg_port or not data_port:
        print("Error initializing serial ports. Exiting...")
        exit()

    print("Sending configuration commands...")
    send_config(cfg_port, cli_commands)


    #Read data sent via serail UART to terminal TODO: update to parse whole packet
    try:
        print("Reading data from radar...")
        while True:
            if data_port.in_waiting > 0:
                data = data_port.read(data_port.in_waiting)
                data_array = np.frombuffer(data, dtype=np.uint8)
                print(data_array)
            
    except KeyboardInterrupt:
        print("\nTerminating...")
    finally:
        cfg_port.close()
        data_port.close()
        print("Ports closed. Exiting...")

