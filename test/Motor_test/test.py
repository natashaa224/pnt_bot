import time
import serial
motor_port0 = "/dev/ttyACM0"
# motor_port1 = "/dev/BACK_LEFT"
# motor_port2 = "/dev/FRONT_RIGHT"
# motor_port3 = "/dev/BACK_RIGHT"

slave_1 = '\x01' 
slave_2 = '\x02' 
slave_3 = '\x03' 

add_pos = '\x60\x2C'
m_read = '\x03'

ser = serial.Serial(port=motor_port0, baudrate=9600, timeout=1, parity=serial.PARITY_NONE,
                            bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
# ser1 = serial.Serial(port=motor_port1, baudrate=57600, timeout=1, parity=serial.PARITY_NONE,
#                             bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
# ser2 = serial.Serial(port=motor_port2, baudrate=57600, timeout=1, parity=serial.PARITY_NONE,
#                             bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)
# ser3 = serial.Serial(port=motor_port3, baudrate=57600, timeout=1, parity=serial.PARITY_NONE,
#                             bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE)


def tohex(val, shift):
    return hex((val + (1 << shift)) % (1 << shift))

def alarm_chk(motor, data):
    global alarm_status
    if data == '0000':              # read 3 digits
        alarm_status = False
    elif data == '00e1' or data == '00e0':
        alarm_status = True
        message = "ERROR: Over Current!"
    elif data == '0100' or data == '0101':
        alarm_status = True
        message = "ERROR: Over Load!"
    elif data == '0102':
        alarm_status = True
        message = "ERROR: Motor Rotor Blocked"
    elif data == '0180':
        alarm_status = True
        message = "ERROR: Position Error Over-Large!"
    elif data == '01a0' or data == '00a0': #0a0
        alarm_status = True
        message = "ERROR: Over Speed!"
    elif data == '01a1' or data == '00a1': #0a1
        alarm_status = True
        message = "ERROR: Speed Out of Control!"
    elif data == '00d0':
        alarm_status = True
        message = "ERROR: DC Bus Under Voltage!"
    elif data == '00c0':
        alarm_status = True
        message = "ERROR: DC Bus Over Voltage!"
    elif data == '0171' or data == '0172':
        alarm_status = True
        message = "ERROR: Encoder Parameters Read Error!"
    elif data == '0190':
        message = "ERROR: Excessive Vibration!"
        alarm_status = True
    elif data == '0150':
        message = "ERROR: Encoder Line Breaked!"
        alarm_status = True
    elif data == '0151' or data == '0170':
        message = "ERROR: Encoder Data Error!"
        alarm_status = True
    elif data == '0152':
        message = "ERROR: Initilized Position of Encoder Error!"
        alarm_status = True
    elif data == '0240':
        message = "ERROR: CRC Verification Error When EEPROM Parameter is Saved!"
        alarm_status = True
    elif data == '0570':
        message = "ERROR: Forced alarm input!"
        alarm_status = True
    elif data == '0120':
        message = "ERROR: Resistance Dischage Circuit Over-Load!"
        alarm_status = True
    elif data == '0153':
        message = "ERROR: Encoder Battery Error!"
        alarm_status = True
    elif data == '0210' or data == '0211' or data == '0212':
        message = "ERROR: I/F Input Interface Allocation Error!"
        alarm_status = True
    else:
        message = "ERROR: Unknown Error " + data + \
            " in Motor: Unknown Error!!!"
        alarm_status = True
    if alarm_status:
        print(message)
    return alarm_status

def crc_check(data):
    calc_crc = crc16(data[:-2])
    crc = data[-2:]
    crc = crc[0] + crc[1]
    if calc_crc == crc:
        return True
    else:
        print("ERROR:  " + "motor" + " Motor: CRC Check Failed!")
        return False

def crc16(data):
    '''
    CRC-16-ModBus Algorithm
    '''
    data = bytearray(data,encoding="iso-8859-1")
    poly = 0xA001
    crc = 0xFFFF
    for b in data:
        crc ^= (0xFF & b)
        for _ in range(0, 8):
            if (crc & 0x0001):
                crc = ((crc >> 1) & 0xFFFF) ^ poly
            else:
                crc = ((crc >> 1) & 0xFFFF)
    crc = tohex(crc, 16)
    byte = '0xff'
    d1 = int(tohex((int(crc, 16) & int(byte, 16)), 16), 16)
    d2 = int(tohex((int(crc, 16) >> 8), 16), 16)
    data = chr(d1) + chr(d2)
    return data

def alarm_clear(motor):
    global alarm_status
    print("INFO:   clearing Alarm in " + "motor" + " Motor")
    alrm_clr = '\x01' + '\x06' + '\x00\x33' + '\x11\x11'
    crc = crc16(alrm_clr)
    alrm_clr = alrm_clr + crc
    # ser.write(bytes(alrm_clr))
    ser.write(bytes(alrm_clr,'iso-8859-1'))
    time.sleep(0.03)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read().decode("iso8859-1")
        if crc_check(out):
            print(str(out))
            if out == alrm_clr:
                print("INFO:   Alarm Cleared in " +
                              "right" + " Motor")
                alarm_status = False
                return alarm_status
        return False
    except Exception as e:
        print(e)
        return False

def alarm_read():
    global alarm
    get_alrm = '\x01' + '\x03' + '\x0B\x03' + '\x00\x01'
    crc = crc16(get_alrm)
    get_alrm = get_alrm + crc
    print(f"Write data:{bytes(get_alrm,'iso-8859-1')}")
    ser.write(bytes(get_alrm,'iso-8859-1'))
    time.sleep(0.3)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read().decode("iso-8859-1")
        if crc_check(out):
            print(f"output:{bytes(out,'iso-8859-1')}")
            print(f"output[3]:{bytes(out[3],'iso-8859-1')}")
            print(f"output[4]:{bytes(out[4],'iso-8859-1')}")           
            value3 = out[3]
            value4 = out[4]
            hex_value3 = hex(ord(value3))[2:].zfill(2)
            # print(f"hexval3:{hex_value3}")
            hex_value4 = hex(ord(value4))[2:].zfill(2)
            # print(f"hexval4:{hex_value4}")
            data = hex_value3 + hex_value4
            print(data)
            alarm = alarm_chk(out[0], data)
            if alarm: 
                # alarm = alarm_clear('\x01')
                print("MOTOR'S STOPED ALARM DITECTED!!") 
    except Exception as e:
        print(e)
        pass

def pos_read(slave):
    get_pos = slave + m_read + add_pos + '\x00\x02'
    crc = crc16(get_pos)
    get_pos = get_pos + crc
    print(f"{bytes(get_pos,'iso-8859-1')}")
    ser.write(bytes(get_pos,'iso-8859-1'))
    time.sleep(0.05)
    out = ''
    try:
        while ser.inWaiting():
            out += ser.read().decode("iso8859-1")
            # print(f"output:{bytes(out,'iso-8859-1')}")
            # print(f"output[3]:{bytes(out[3],'iso-8859-1')}")
            # print(f"output[4]:{bytes(out[4],'iso-8859-1')}")  
        if crc_check(out):
            ticks = ord(out[3])*16**6 + ord(out[4])*16**4 + \
                ord(out[5])*16**2 + ord(out[6])
            position = -(ticks & 0x80000000) | (ticks & 0x7fffffff)
            print(position)
    except Exception as e:
        print(e)

def command_pos_read(serial):
    global alarm
    get_alrm = '\x01' + '\x03' + '\x04\x05' + '\x00\x02' 

    crc = crc16(get_alrm)
    get_alrm = get_alrm + crc
    print(f"Write data:{bytes(get_alrm,'iso-8859-1')}")
    serial.write(bytes(get_alrm,'iso-8859-1'))
    time.sleep(0.1)
    out = ''
    try:
        while serial.inWaiting():
            out += serial.read().decode("iso-8859-1")
        if crc_check(out):
            print(f"output:{bytes(out,'iso-8859-1')}")
            print(f"output[3]:{bytes(out[3],'iso-8859-1')}")
            print(f"output[4]:{bytes(out[4],'iso-8859-1')}")           
            value3 = out[3]
            value4 = out[4]
            hex_value3 = hex(ord(value3))[2:].zfill(2)
            # print(f"hexval3:{hex_value3}")
            hex_value4 = hex(ord(value4))[2:].zfill(2)
            # print(f"hexval4:{hex_value4}")
            data = hex_value3 + hex_value4
            print(data)
            alarm = alarm_chk(out[0], data)
            if alarm: 
                # alarm = alarm_clear('\x01')
                print("MOTOR'S STOPED ALARM DITECTED!!") 
    except Exception as e:
        print(e)
        pass


def single_read(serial,get_alrm):
    global alarm
    # get_alrm = '\x01' + '\x06' + '\x04\x05' + '\x00\x83' - enable motor
    # get_alrm = '\x01' + '\x06' + '\x04\x05' + '\x00\x03' ##- disable motor
    # get_alrm = '\x01' + '\x03' + '\x04\x05' + '\x00\x02' #- read data

    crc = crc16(get_alrm)
    get_alrm = get_alrm + crc
    print(f"Write data:{bytes(get_alrm,'iso-8859-1')}")
    serial.write(bytes(get_alrm,'iso-8859-1'))
    time.sleep(0.1)
    out = ''
    try:
        while serial.inWaiting():
            out += serial.read().decode("iso-8859-1")
        if crc_check(out):
            print(f"output:{bytes(out,'iso-8859-1')}")
            print(f"output[3]:{bytes(out[3],'iso-8859-1')}")
            print(f"output[4]:{bytes(out[4],'iso-8859-1')}")           
            value3 = out[3]
            value4 = out[4]
            hex_value3 = hex(ord(value3))[2:].zfill(2)
            # print(f"hexval3:{hex_value3}")
            hex_value4 = hex(ord(value4))[2:].zfill(2)
            # print(f"hexval4:{hex_value4}")
            data = hex_value3 + hex_value4
            print(data)
            alarm = alarm_chk(out[0], data)
            if alarm: 
                # alarm = alarm_clear('\x01')
                print("MOTOR'S STOPED ALARM DITECTED!!") 
            
    except Exception as e:
        print(e)
        pass

# data = '\x01' + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x03' + '\xE8' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##1000 RPM

data_1 = slave_1 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM
data_11 = slave_1 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM

data_2 = slave_2 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM
data_22 = slave_2 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM

# data_3 = slave_3 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM
# data_33 = slave_3 + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##0 RPM

# data = '\x01' + '\x10' + '\x62' + '\x00' + '\x00' + '\x08' + '\x10' + '\x00' + '\x02' + '\x00' + '\x00' + '\x00' + '\x00' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x64' + '\x00' + '\x00' + '\x00' + '\x10' ##100 RPM

# data = '\x01' + '\x03' + '\x60' + '\x2C' + '\x00' + '\x02'

single_read(ser,data_1)
time.sleep(1)
pos_read(slave_1)
single_read(ser,data_11)
time.sleep(0.5)
single_read(ser,data_2)
time.sleep(1)
pos_read(slave_2)
single_read(ser,data_22)
time.sleep(0.5)
# single_read(ser,data_3)
# time.sleep(1)
# pos_read(slave_3)
# single_read(ser,data_33)
# time.sleep(0.5)

