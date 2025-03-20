import struct
import time
import numpy as np
import serial
import codecs  
import binascii 
import math
import os
import time  

# -------------------Configuration-----------------------------------
CFG_FILE = "/home/lorien/dev/capstone/ProjectStarshotRadarWrapper/Cfgs/testcfg.cfg"
COM_PORTS = {"cfg": "/dev/ttyUSB0", "data": "/dev/ttyUSB1"} # going to change this to linux friendly on pi 
BAUDRATE_READ = 921600
BAUDRATE_WRITE = 115200
PIPE_NAME = "/home/lorien/dev/capstone/radarpipe"
#---------------------End of Configuration---------------------------


#-----------------Magic Word---------------------------
MAGIC_WORD = [2, 1, 4, 3, 6, 5, 8, 7]
TC_PASS   =  0
TC_FAIL   =  1
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


#----------------Start of Packet Parsing---------------------
def checkMagicPattern(data):
    try:
        """!
        This function check if data arrary contains the magic pattern which is the start of one mmw demo output packet.  

            @param data : 1-demension byte array
            @return     : 1 if magic pattern is found
                        0 if magic pattern is not found 
        """ 
        found = 0
        if (data[0] == 2 and data[1] == 1 and data[2] == 4 and data[3] == 3 and data[4] == 6 and data[5] == 5 and data[6] == 8 and data[7] == 7):
            found = 1
        return (found)
    except Exception:
        pass


def parse_gtrack_targets(payload):
    """Parses GTRACK target data from the payload."""
    target_size = 40  # Each target data block is 40 bytes
    num_targets = len(payload) // target_size
    targets = []
    for i in range(num_targets):
        offset = i * target_size
        tid = struct.unpack('<I', payload[offset:offset + 4])[0]
        posX, posY, posZ = struct.unpack('<fff', payload[offset + 4:offset + 16])
        velX, velY, velZ = struct.unpack('<fff', payload[offset + 16:offset + 28])
        accX, accY, accZ = struct.unpack('<fff', payload[offset + 28:offset + 40])
        targets.append({
            'tid': tid,
            'posX': posX, 'posY': posY, 'posZ': posZ,
            'velX': velX, 'velY': velY, 'velZ': velZ,
            'accX': accX, 'accY': accY, 'accZ': accZ
        })
    return targets
def parser_helper(data, readNumBytes,debug=False):
    """!
       This function is called by parser_one_mmw_demo_output_packet() function or application to read the input buffer, find the magic number, header location, the length of frame, the number of detected object and the number of TLV contained in this mmw demo output packet.

        @param data                   : 1-demension byte array holds the the data read from mmw demo output. It ignorant of the fact that data is coming from UART directly or file read.  
        @param readNumBytes           : the number of bytes contained in this input byte array  
            
        @return headerStartIndex      : the mmw demo output packet header start location
        @return totalPacketNumBytes   : the mmw demo output packet lenght           
        @return numDetObj             : the number of detected objects contained in this mmw demo output packet          
        @return numTlv                : the number of TLV contained in this mmw demo output packet           
        @return subFrameNumber        : the sbuframe index (0,1,2 or 3) of the frame contained in this mmw demo output packet
    """ 
    

    headerStartIndex = -1

    for index in range (readNumBytes):
        #if data[index:index+8:1].__len__() == 8:
            if checkMagicPattern(data[index:index+8:1]) == 1:
                headerStartIndex = index
                break
    
    if headerStartIndex == -1: # does not find the magic number i.e output packet header 
        totalPacketNumBytes = -1
        numDetObj           = -1
        numTlv              = -1
        subFrameNumber      = -1
        platform            = -1
        frameNumber         = -1
        timeCpuCycles       = -1
    else: # find the magic number i.e output packet header 
        totalPacketNumBytes = getUint32(data[headerStartIndex+12:headerStartIndex+16:1])
        platform            = getHex(data[headerStartIndex+16:headerStartIndex+20:1])
        frameNumber         = getUint32(data[headerStartIndex+20:headerStartIndex+24:1])
        timeCpuCycles       = getUint32(data[headerStartIndex+24:headerStartIndex+28:1])
        numDetObj           = getUint32(data[headerStartIndex+28:headerStartIndex+32:1])
        numTlv              = getUint32(data[headerStartIndex+32:headerStartIndex+36:1])
        subFrameNumber      = getUint32(data[headerStartIndex+36:headerStartIndex+40:1])
        
    if(False):
        print("headerStartIndex    = %d" % (headerStartIndex))
        print("totalPacketNumBytes = %d" % (totalPacketNumBytes))
        print("platform            = %s" % (platform)) 
        print("frameNumber         = %d" % (frameNumber)) 
        print("timeCpuCycles       = %d" % (timeCpuCycles))   
        print("numDetObj           = %d" % (numDetObj)) 
        print("numTlv              = %d" % (numTlv))
        print("subFrameNumber      = %d" % (subFrameNumber))   
                            
    return (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber)





def parser_one_mmw_demo_output_packet(data, readNumBytes, debug=False):
    """!
       Parses a single mmWave demo output packet, handling TLVs for detected points, GTRACK targets, and GTRACK point clouds.

        @param data: 1-dimensional byte array from mmWave radar
        @param readNumBytes: Total number of bytes in the data array
        @param debug: Boolean flag for enabling debug prints

        @return result: Pass or fail status
        @return headerStartIndex: Header start index in the packet
        @return totalPacketNumBytes: Total packet length in bytes
        @return numDetObj: Number of detected objects
        @return numTlv: Number of TLVs in the packet
        @return subFrameNumber: Sub-frame index of the packet
        @return detected_targets: List of GTRACK targets parsed
        @return point_cloud: List of point cloud data parsed
    """
    headerNumBytes = 44
    PI = 3.14159265

    detected_targets = []
    point_cloud = []

    result = TC_PASS

    # Call parser_helper() to locate packet header and retrieve metadata
    (headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber) = parser_helper(data, readNumBytes, debug)

    if headerStartIndex == -1:
        result = TC_FAIL
        print("************ Frame Fail: Cannot find the magic words ************")
        return result, -1, -1, -1, -1, -1, detected_targets, point_cloud

    nextHeaderStartIndex = headerStartIndex + totalPacketNumBytes

    # Check for incomplete packets
    if headerStartIndex + totalPacketNumBytes > readNumBytes:
        result = TC_FAIL
        print("********** Frame Fail: Insufficient bytes in buffer ***********")
        return result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detected_targets, point_cloud

    # Start processing TLVs
    tlvStart = headerStartIndex + headerNumBytes
    for tlvIndex in range(numTlv):
        tlvType = getUint32(data[tlvStart:tlvStart + 4])
        tlvLen = getUint32(data[tlvStart + 4:tlvStart + 8])
        tlvPayload = data[tlvStart + 8:tlvStart + 8 + tlvLen]

        if False:
            print(f"TLV {tlvIndex + 1}: Type={tlvType}, Length={tlvLen}")

        # Handle detected points
        if tlvType == 1:  # MMWDEMO_UART_MSG_DETECTED_POINTS
            ...
            #print("")

        # Handle GTRACK Targets
        elif tlvType == 10:  # GTRACK Targets (Type 10)
            targets = parse_gtrack_targets(tlvPayload)
            detected_targets.extend(targets)
            if debug:
                for target in targets:
                    print(target)
                    print(f"Target {target['tid']}: pos=({target['posX']:.2f}, {target['posY']:.2f}, {target['posZ']:.2f}), "
                          f"vel=({target['velX']:.2f}, {target['velY']:.2f}, {target['velZ']:.2f}), "
                          f"acc=({target['accX']:.2f}, {target['accY']:.2f}, {target['accZ']:.2f})")

  
        else:
            ...
            #print(f"Unhandled TLV Type: {tlvType}")

        # Move to the next TLV
        tlvStart += 8 + tlvLen
    return result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detected_targets, point_cloud




#-------------------Serial port init-------------------------
def init_serial_port(port, baudrate):
    """Initializes a serial port for communication."""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Opened port {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening port {port}: {e}")
        return None
#-------------------End of Serial port init-----------------------





def getUint32(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer.

        @param data : 1-demension byte array  
        @return     : 32-bit unsigned integer
    """ 
    if len(data) == 0:
        return None
    return (data[0] +
            data[1]*256 +
            data[2]*65536 +
            data[3]*16777216)

def getUint16(data):
    """!
       This function coverts 2 bytes to a 16-bit unsigned integer.

        @param data : 1-demension byte array
        @return     : 16-bit unsigned integer
    """ 
    return (data[0] +
            data[1]*256)

def getHex(data):
    """!
       This function coverts 4 bytes to a 32-bit unsigned integer in hex.

        @param data : 1-demension byte array
        @return     : 32-bit unsigned integer in hex
    """         
    #return (binascii.hexlify(data[::-1]))
    word = [1, 2**8, 2**16, 2**24]
    return np.matmul(data,word)



def write_to_pipe(data):
    """Writes parsed object data to the named pipe."""
    try:
        print("In send to pipe")
        if not os.path.exists(PIPE_NAME):
            print(f"Pipe {PIPE_NAME} does not exist yet. Retrying...")
            time.sleep(1)
            return  # Don't proceed with writing, just return and retry

        with open(PIPE_NAME, "w") as pipe:
            pipe.write(data + "\n")
            pipe.flush()
    except FileNotFoundError:
        print("Pipe not found, waiting...")
        time.sleep(1)
    except BrokenPipeError:
        print("Broken pipe, retrying...")
        time.sleep(1)



if __name__ == "__main__":
    #Pass CFG commands to IWR6843ISK-ODS 
    cli_commands = parse_cfg_file(CFG_FILE) 
    if not cli_commands:
        print("No valid configuration commands found. Exiting...")
        exit()

    cfg_port = init_serial_port(COM_PORTS["cfg"],BAUDRATE_WRITE)
    data_port = init_serial_port(COM_PORTS["data"],BAUDRATE_READ)

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
                readNumBytes = len(data_array)
                result, headerStartIndex, totalPacketNumBytes, numDetObj, numTlv, subFrameNumber, detected_targets, point_cloud = parser_one_mmw_demo_output_packet(data_array, readNumBytes)

                # if detected objects are valid, we write them to the named pipe
                if detected_targets:

                       for target in detected_targets: 
                            msg = (f"{target['tid']}|"
                                    f"{target['posX']:.2f}|{target['posY']:.2f}|{target['posZ']:.2f}|"
                                    f"{target['velX']:.2f}|{target['velY']:.2f}|{target['velZ']:.2f}|"
                                    f"{target['accX']:.2f}|{target['accY']:.2f}|{target['accZ']:.2f}")
                            print("Writing to pipe:", msg) 
                            write_to_pipe(msg) # to C++ motor process. 



                time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nTerminating...")
    
    finally:
        cfg_port.close()
        data_port.close()
        print("Ports closed. Exiting...")