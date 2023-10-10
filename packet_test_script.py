import serial
import crcmod
import struct
import sys

PORT = 'COM3'
BAUDRATE = 10000
TIMEOUT = 0.1

HEADER_FMT = '<chcc'

seq = 0

def c_char(n: int):
    return bytes([n])

def create_packet_header(data_length: int):
    global seq

    header = struct.pack(
        HEADER_FMT,
        c_char(0xA5),
        data_length,
        c_char(seq),
        c_char(0),
    )

    if seq == 255:
        seq = 0
    else:
        seq += 1

    return header

def main():
    print('[SERIAL PROTOCOL TESTING]', end='\n\n')
    
    crc8 = crcmod.Crc(0xFF)

    # print(crc8(create_packet_header(10)));

    test_physical = len(sys.argv) > 1 and sys.argv[1] == 'test-phys'

    if test_physical:
        arduino = serial.Serial(port=PORT, baudrate=BAUDRATE, timeout=TIMEOUT)

    while True:
        cmd = input('Enter dummy packet type to send to Arduino (dr16, q=quit): ')
        if cmd == 'q':
            print('quitting...')
            break;
        elif cmd == 'dr16':
            header = create_packet_header(10)
            if test_physical:
                arduino.write(header)
        else:
            continue
        
        print('\n[ARDUINO READBACK]', end='\n')
        line = arduino.readline().decode('utf-8')
        while line != '':
            print(line, end='')
            line = arduino.readline().decode('utf-8')

        print('\n', end='')

if __name__ == '__main__':
    main()