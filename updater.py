import serial
from optparse import OptionParser
from time import sleep


def waitForChar(ser, c):
    recv_char = ser.read()
    while recv_char not in c:
        recv_char = ser.read()
    return recv_char


def calcStmCrc(data, idx, len):
    cnt = 0
    crc = 0xffffffff

    while cnt < len:
        word = data[idx] | (data[idx+1] << 8) | (data[idx+2]
                                                 << 16) | (data[idx+3] << 24)
        cnt = cnt + 4
        idx = idx + 4

        crc = crc ^ word

        for i in range(0, 32):
            if crc & 0x80000000:
                # Polynomial used in STM32
                crc = ((crc << 1) ^ 0x04C11DB7) & 0xffffffff
            else:
                crc = (crc << 1) & 0xffffffff
    return crc


PAGE_SIZE_BYTES = 1024


parser = OptionParser()
parser.add_option("-f", "--file", dest="filename",
                  help="update file")
parser.add_option("-d", "--device", dest="device",
                  help="serial interface")

(options, args) = parser.parse_args()

if not options.filename:   # if filename is not given
    parser.error('Filename not given')
    exit()
if not options.device:   # if device is not given
    parser.error('Device not given')
    exit()
ser = serial.Serial(options.device, 115200, timeout=2, stopbits=2)

updateFile = open(options.filename, "rb")
data = bytearray(updateFile.read())
updateFile.close()

numBytes = len(data)
numPages = (numBytes + PAGE_SIZE_BYTES - 1) // PAGE_SIZE_BYTES

while (len(data) % PAGE_SIZE_BYTES) > 0:
    data.append(0)

print("File length is %d bytes/%d pages" % (numBytes, numPages))
print("Resetting device...")

ser.write(b'reset\r')
version = waitForChar(ser, b'S2')

if version == b'2':
    print("Version 2 bootloader, sending magic")
    ser.write(b'\xAA')
    waitForChar(ser, b'S')

print("Sending number of pages...")

ser.write([numPages])

waitForChar(ser, b'P')

done = False
page = 0
idx = 0

while not done:
    crc = calcStmCrc(data, idx, PAGE_SIZE_BYTES)
    c = 0

    while c != b'P' and not done:
        print("Sending page %d..." % (page), end=' ')
        idx = page * PAGE_SIZE_BYTES
        cnt = 0

        while cnt < PAGE_SIZE_BYTES:
            ser.write([data[idx]])
            idx = idx + 1
            cnt = cnt + 1

        c = ser.read()

        if b'C' == c:
            ser.write([crc & 0xFF])
            ser.write([(crc >> 8) & 0xFF])
            ser.write([(crc >> 16) & 0xFF])
            ser.write([(crc >> 24) & 0xFF])

            c = ser.read()

        if b'D' == c:
            print("CRC correct!")
            print("Update done!")
            done = True
        elif b'E' == c:
            print("CRC error!")
            waitForChar(ser, b'T')
        elif b'P' == c:
            print("CRC correct!")
            page = page + 1
        elif b'T' == c:
            print("Sync Error!")
        else:
            print(c)
