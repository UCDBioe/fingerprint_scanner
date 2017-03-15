'''
Created on 2/2/2017

@author: Steven Lammers <StevenLammers12@gmail.com>
'''
import serial
import pdb
import time
import binascii

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1) 

UseSerialDebug = True


def delay(seconds):
    '''
    wait a number of secons 
    '''
    time.sleep(seconds)


def Open():
    '''
        Initialises the device and gets ready for commands
    '''
    delay(0.1)
    cp = Command_Packet('Open',UseSerialDebug)
    cp.ParameterFromInt(1)
    packetbytes = cp.GetPacketBytes()
    #print 'Command packetbytes: '+ packetbytes
    #pdb.set_trace()
    SendCommand(packetbytes, 12)
    delay(.5)
    rp = GetResponse()

    rp_ACK = rp.ACK

    del cp
    del rp
    del packetbytes
    return rp_ACK


def Close():
    '''
         Does not actually do anything (according to the datasheet)
         I implemented open, so had to do closed too... lol
    '''
    cp = Command_Packet('Close',UseSerialDebug=self.UseSerialDebug)
    cp.Parameter[0] = 0x00;
    cp.Parameter[1] = 0x00;
    cp.Parameter[2] = 0x00;
    cp.Parameter[3] = 0x00;
    packetbytes = cp.GetPacketBytes()
    SendCommand(packetbytes, 12)
    rp = GetResponse()
    if not ser is None:
        ser.close()
    del cp
    del rp
    del packetbytes
    return rp_ACK



def SetLED(on=True):
    '''
         Turns on or off the LED backlight
         LED must be on to see fingerprints
         Parameter: true turns on the backlight, false turns it off
         Returns: True if successful, false if not
    '''
    cp = Command_Packet('CmosLed',UseSerialDebug=UseSerialDebug)
    cp.Parameter[0] = 0x01 if on else 0x00;
    cp.Parameter[1] = 0x00;
    cp.Parameter[2] = 0x00;
    cp.Parameter[3] = 0x00;
    packetbytes = cp.GetPacketBytes()
    SendCommand(packetbytes, 12)
    delay(2)
    rp = GetResponse()
    # Print response
    while not rp.ACK:
        print('looping open')
        ser.flush()
        SendCommand(packetbytes, 12)
        delay(2)
        rp = GetResponse()

    retval = rp.ACK
    #SL should I del cp after use ??????????????
    del cp
    del rp
    del packetbytes
    return retval


def serializeToSend(bytearr):
    return ' '.join(binascii.hexlify(ch) for ch in bytes(bytearr))


def SendCommand(cmd,length):
    '''
         resets the Data_Packet class, and gets ready to download
         Not implemented due to memory restrictions on the arduino
         may revisit this if I find a need for it
        void StartDataDownload();

         Returns the next data packet 
         Not implemented due to memory restrictions on the arduino
         may revisit this if I find a need for it
        Data_Packet GetNextDataPacket();
    '''
    ser.write(bytes(cmd))
    if UseSerialDebug:
        print serializeToSend(cmd)
        #print bytes(cmd)
        #print repr(bytes(cmd))[1:-1]


def GetResponse():
    '''
    Gets the response to the command from the software serial channel (and waits for it)
    '''
    interval = 0.1
    delay(interval)
    r = bytearray(ser.read(ser.inWaiting()))
    #r = bytearray(ser.read(12))
    # Print buffer
    #print 'print buffer: %s'% self.serializeToSend(r)
    #print 'print buffer raw: ' + r
    #pdb.set_trace()

    #print 'RESPONSE'
    #for i in r:
    #  print 'r: %s'% hex(i)

    print 'Packet size: %i'% len(r)
    rp = Response_Packet(r, UseSerialDebug)

    if rp.ACK:
        delay(interval)
        r2 = bytearray(ser.read(ser.inWaiting()))
        rp2 = Response_Packet(r2,UseSerialDebug)
        while str(rp2._lastBuffer).__len__()>0:
            rp.RawBytes.extend(rp2.RawBytes)
            rp._lastBuffer += rp2._lastBuffer
            delay(interval)
            r2 = bytearray(ser.read(ser.inWaiting()))
            rp2 = Response_Packet(r2,UseSerialDebug)
    return rp


class Packet:
    '''
        Generic Internal Packet Class
    '''
    COMMAND_START_CODE_1 = 0x55;    # Static byte to mark the beginning of a command packet    -    never changes
    COMMAND_START_CODE_2 = 0xAA;    # Static byte to mark the beginning of a command packet    -    never changes
    COMMAND_DEVICE_ID_1  = 0x01;    # Device ID Byte 1 (lesser byte)                            -    theoretically never changes
    COMMAND_DEVICE_ID_2  = 0x00;    # Device ID Byte 2 (greater byte)                            -    theoretically never changes
    
    def GetHighByte(self, w):
        '''
        Returns the high byte from a word
        '''
        return (w>>8)&0x00FF
    
    def GetLowByte(self, w):
        '''
        Returns the low byte from a word        
        '''
        return w&0x00FF
    
    def CalculateCheckSum(self,bytearr):
        return sum(map(ord,bytes(bytearr)))
    
    def serializeToSend(self,bytearr):
        return ' '.join(binascii.hexlify(ch) for ch in bytes(bytearr))



class Command_Packet(Packet):
    '''
        Command Packet Class
        Used to build the serial message
    '''
    
    command = bytearray(2)
    cmd = ''
    commands = {
                    'NotSet'                  : 0x00,        # Default value for enum. Scanner will return error if sent this.
                    'Open'                    : 0x01,        # Open Initialization
                    'Close'                   : 0x02,        # Close Termination
                    'UsbInternalCheck'        : 0x03,        # UsbInternalCheck Check if the connected USB device is valid
                    'ChangeBaudrate'          : 0x04,        # ChangeBaudrate Change UART baud rate
                    'SetIAPMode'              : 0x05,        # SetIAPMode Enter IAP Mode In this mode, FW Upgrade is available
                    'CmosLed'                 : 0x12,        # CmosLed Control CMOS LED
                    'GetEnrollCount'          : 0x20,        # Get enrolled fingerprint count
                    'CheckEnrolled'           : 0x21,        # Check whether the specified ID is already enrolled
                    'EnrollStart'             : 0x22,        # Start an enrollment
                    'Enroll1'                 : 0x23,        # Make 1st template for an enrollment
                    'Enroll2'                 : 0x24,        # Make 2nd template for an enrollment
                    'Enroll3'                 : 0x25,        # Make 3rd template for an enrollment, merge three templates into one template, save merged template to the database
                    'IsPressFinger'           : 0x26,        # Check if a finger is placed on the sensor
                    'DeleteID'                : 0x40,        # Delete the fingerprint with the specified ID
                    'DeleteAll'               : 0x41,        # Delete all fingerprints from the database
                    'Verify1_1'               : 0x50,        # Verification of the capture fingerprint image with the specified ID
                    'Identify1_N'             : 0x51,        # Identification of the capture fingerprint image with the database
                    'VerifyTemplate1_1'       : 0x52,        # Verification of a fingerprint template with the specified ID
                    'IdentifyTemplate1_N'     : 0x53,        # Identification of a fingerprint template with the database
                    'CaptureFinger'           : 0x60,        # Capture a fingerprint image(256x256) from the sensor
                    'MakeTemplate'            : 0x61,        # Make template for transmission
                    'GetImage'                : 0x62,        # Download the captured fingerprint image(256x256)
                    'GetRawImage'             : 0x63,        # Capture & Download raw fingerprint image(320x240)
                    'GetTemplate'             : 0x70,        # Download the template of the specified ID
                    'SetTemplate'             : 0x71,        # Upload the template of the specified ID
                    'GetDatabaseStart'        : 0x72,        # Start database download, obsolete
                    'GetDatabaseEnd'          : 0x73,        # End database download, obsolete
                    'UpgradeFirmware'         : 0x80,        # Not supported
                    'UpgradeISOCDImage'       : 0x81,        # Not supported
                    'Ack'                     : 0x30,        # Acknowledge.
                    'Nack'                    : 0x31         # Non-acknowledge
                }
    

    def __init__(self,*args,**kwargs):
        '''
            Command Packet Constructor
        '''
        commandName=args[0]
        kwargs.setdefault('UseSerialDebug', True)
        self.UseSerialDebug= kwargs['UseSerialDebug']
        if self.UseSerialDebug:
            print 'Command: %s' % commandName
        self.cmd = self.commands[commandName]
        
    UseSerialDebug = True
    Parameter = bytearray(4)
    
    
    
    def GetPacketBytes(self):
        '''
        returns the 12 bytes of the generated command packet
        remember to call delete on the returned array
        '''
        
        self.command[0] = self.GetLowByte(self.cmd)
        self.command[1] = self.GetHighByte(self.cmd)
        
        packetbytes= bytearray(12)
        packetbytes[0] = self.COMMAND_START_CODE_1
        packetbytes[1] = self.COMMAND_START_CODE_2
        packetbytes[2] = self.COMMAND_DEVICE_ID_1
        packetbytes[3] = self.COMMAND_DEVICE_ID_2
        packetbytes[4] = self.Parameter[0]
        packetbytes[5] = self.Parameter[1]
        packetbytes[6] = self.Parameter[2]
        packetbytes[7] = self.Parameter[3]
        packetbytes[8] = self.command[0]
        packetbytes[9] = self.command[1]
        chksum = self.CalculateCheckSum(packetbytes[0:9])
        packetbytes[10] = self.GetLowByte(chksum)
        packetbytes[11] = self.GetHighByte(chksum)

        return packetbytes;        
    
    def ParameterFromInt(self, i):
        '''
        Converts the int to bytes and puts them into the paramter array
        '''
        
        self.Parameter[0] = (i & 0x000000ff);
        self.Parameter[1] = (i & 0x0000ff00) >> 8;
        self.Parameter[2] = (i & 0x00ff0000) >> 16;
        self.Parameter[3] = (i & 0xff000000) >> 24;



class Response_Packet(Packet):
    '''
        Response Packet Class
    '''
    
    errors = {
                    'NO_ERROR'                      : 0x0000,    # Default value. no error
                    'NACK_TIMEOUT'                  : 0x1001,    # Obsolete, capture timeout
                    'NACK_INVALID_BAUDRATE'         : 0x1002,    # Obsolete, Invalid serial baud rate
                    'NACK_INVALID_POS'              : 0x1003,    # The specified ID is not between 0~199
                    'NACK_IS_NOT_USED'              : 0x1004,    # The specified ID is not used
                    'NACK_IS_ALREADY_USED'          : 0x1005,    # The specified ID is already used
                    'NACK_COMM_ERR'                 : 0x1006,    # Communication Error
                    'NACK_VERIFY_FAILED'            : 0x1007,    # 1:1 Verification Failure
                    'NACK_IDENTIFY_FAILED'          : 0x1008,    # 1:N Identification Failure
                    'NACK_DB_IS_FULL'               : 0x1009,    # The database is full
                    'NACK_DB_IS_EMPTY'              : 0x100A,    # The database is empty
                    'NACK_TURN_ERR'                 : 0x100B,    # Obsolete, Invalid order of the enrollment (The order was not as: EnrollStart -> Enroll1 -> Enroll2 -> Enroll3)
                    'NACK_BAD_FINGER'               : 0x100C,    # Too bad fingerprint
                    'NACK_ENROLL_FAILED'            : 0x100D,    # Enrollment Failure
                    'NACK_IS_NOT_SUPPORTED'         : 0x100E,    # The specified command is not supported
                    'NACK_DEV_ERR'                  : 0x100F,    # Device Error, especially if Crypto-Chip is trouble
                    'NACK_CAPTURE_CANCELED'         : 0x1010,    # Obsolete, The capturing is canceled
                    'NACK_INVALID_PARAM'            : 0x1011,    # Invalid parameter
                    'NACK_FINGER_IS_NOT_PRESSED'    : 0x1012,    # Finger is not pressed
                    'INVALID'                       : 0XFFFF     # Used when parsing fails          
              }
    
    def __init__(self,_buffer=None,UseSerialDebug=False):
        '''
        creates and parses a response packet from the finger print scanner
        '''
        self.UseSerialDebug= UseSerialDebug
        
        if not (_buffer is None ):
            self.RawBytes = _buffer
            self._lastBuffer = bytes(_buffer)
            if self.UseSerialDebug:
                print 'readed: %s'% self.serializeToSend(_buffer)
            if _buffer.__len__()>=12:
                self.ACK = True if _buffer[8] == 0x30 else False
                self.ParameterBytes[0] = _buffer[4]
                self.ParameterBytes[1] = _buffer[5]
                self.ParameterBytes[2] = _buffer[6]
                self.ParameterBytes[3] = _buffer[7]
                self.ResponseBytes[0]  = _buffer[8]
                self.ResponseBytes[1]  = _buffer[9]
                self.Error = self.ParseFromBytes(self.GetHighByte(_buffer[5]),self.GetLowByte(_buffer[4]))
        
    _lastBuffer = bytes()
    RawBytes = bytearray(12)
    ParameterBytes=bytearray(4)
    ResponseBytes=bytearray(2)
    ACK = False
    Error = None
    UseSerialDebug = True
    
    
    def ParseFromBytes(self,high,low):
        '''
        parses bytes into one of the possible errors from the finger print scanner
        '''
        e  = 'INVALID'
        if high == 0x01:
            if low in self.errors.values():
                errorIndex = self.errors.values().index(low)
                e = self.errors.keys()[errorIndex]
        return e
    
    
    def IntFromParameter(self):
        retval = 0;
        retval = (retval << 8) + self.ParameterBytes[3];
        retval = (retval << 8) + self.ParameterBytes[2];
        retval = (retval << 8) + self.ParameterBytes[1];
        retval = (retval << 8) + self.ParameterBytes[0];
        return retval;




Open()
print ' ' 
print ' ' 
print 'LED ON'
SetLED(True)
delay(3)
print ' ' 
print 'LED OFF'
SetLED(False)
delay(3)
print ' ' 
print 'LED ON'
SetLED(True)
delay(3)
print ' ' 
print 'LED OFF'
SetLED(False)
