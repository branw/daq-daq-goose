import serial
import struct
import math

# Show all packets
VERBOSE = 0

# Native USB port of Due (use Device Manager to find)
PORT = 'COM4'

# File to store output data
OUTPUT_FILE = 'data.txt'

def generate_chirp(t1=2e-3, phi=0, f0=5e3, f1=105e3, num_samples=2048):
    # "Chirpyness" or rate of frequency change
    k = (f1 - f0) / t1

    output = [0] * num_samples
    for i in range(num_samples):
        t = t1 * (i/num_samples)
        # Create a chirp from the frequency f(t) = f0 + kt
        chirp = math.cos(phi + 2*math.pi * (f0*t + k/2 * t*t))
        # Create a Hanning window to envelope the chirp
        window = 0.5 * (1 - math.cos(2*math.pi * i/(num_samples - 1)))
        # Move the signal across a different reference
        output[i] = round(4095/2 + 4095/2 * (chirp * window))
    return output

def read(ser, n):
    data = ser.read(size=n)
    if VERBOSE:
        print('> ' + ''.join('{:02x} '.format(b) for b in data))
    return data

def write(ser, data):
    ser.write(bytearray(data))
    if VERBOSE:
        print('< ' + ''.join('{:02x} '.format(b) for b in data))

def main():
    ser = serial.Serial()
    ser.port = PORT
    ser.baudrate =  115200 # arbitrary
    ser.setRTS(True)
    ser.setDTR(True)
    ser.open()
    
    print('Communicating over port {}'.format(ser.name))

    # Hello
    write(ser, [0, 0, 0, 0, 0])
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x80 or response_len != 2:
        print('unexpected! opcode=0x{:02x}, response_len={}'
               .format(opcode, response_len))
        return

    version = struct.unpack('<BB', read(ser, response_len))
    print('hello: version={}'.format(version))

    '''
    # Queue data
    chirp = generate_chirp()
    queue_data = [0] * (1 + 4 + 4096 + 1)

    queue_data[0] = 1
    queue_data[1] = (4096 >> 0) & 0xff
    queue_data[2] = (4096 >> 8) & 0xff
    queue_data[3] = (4096 >> 16) & 0xff
    queue_data[4] = (4096 >> 24) & 0xff
    for i in range(2048):
        queue_data[5 + 2*i + 0] = (chirp[i] >> 0) & 0xff
        queue_data[5 + 2*i + 1] = (chirp[i] >> 8) & 0xff
    
    write(ser, queue_data)
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x81 or response_len != 0:
        print('unexpected! opcode=0x{:02x}, response_len={:04}'
               .format(opcode, response_len))
        return

    print('queue_data'.format(version))
    '''

    # Initiate data collection
    write(ser, [2, 0, 0, 0, 0])
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x82 or response_len != 0:
        print('unexpected! opcode=0x{:02x}, response_len={}'
               .format(opcode, response_len))
        return

    print('collect_data: started... ', end='')

    # Response that data collection has finished
    opcode, response_len = struct.unpack('<BI', read(ser, 5))
    if opcode != 0x82:
        print('unexpected! opcode=0x{:02x}, response_len={}'
               .format(opcode, response_len))
        return

    print('done! ({} data points)'.format(response_len // 2))

    # Record the data
    raw_data = read(ser, response_len)
    with open(OUTPUT_FILE, 'w') as f:
        for i in range(0, len(raw_data), 2):
            data = raw_data[i] | (raw_data[i + 1] << 8)
            f.write('{}\n'.format(data))
    print('Output data written to {}'.format(OUTPUT_FILE))

    print('Closing connection')
    ser.close()

if __name__ == '__main__':
    main()
