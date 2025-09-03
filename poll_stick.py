import serial
import time
import struct

# === Configuration ===
SERIAL_PORT = "/dev/ttyAMA0"
BAUDRATE = 115200
TIMEOUT = 0.1

MSP_RC = 105  # RC stick/channel data

# === Helper functions ===
def build_msp_request(command, payload=b''):
    header = b"$M<"
    size = len(payload)
    size_byte = size.to_bytes(1, 'little')
    command_byte = command.to_bytes(1, 'little')
    checksum = size
    checksum ^= command
    for b in payload:
        checksum ^= b
    checksum_byte = checksum.to_bytes(1, 'little')
    return header + size_byte + command_byte + payload + checksum_byte

def read_msp_response(ser, expected_payload_size=16):
    data = b''
    start_time = time.time()
    while len(data) < expected_payload_size + 5:  # header(3)+size(1)+cmd(1)+payload+checksum(1)
        if time.time() - start_time > 0.2:
            break
        data += ser.read(ser.in_waiting or 1)
    return data

def parse_msp_rc(response):
    """
    Parse MSP_RC response bytes into a list of channel values.
    """
    if len(response) < 5:
        return []
    size = response[3]
    payload = response[5:5+size]
    channels = []
    for i in range(0, len(payload), 2):
        ch = struct.unpack('<H', payload[i:i+2])[0]
        channels.append(ch)
    return channels

# === Main loop ===
def main():
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
    time.sleep(0.2)
    
    try:
        while True:
            packet = build_msp_request(MSP_RC)
            ser.write(packet)
            time.sleep(0.02)  # short delay for FC to respond
            
            response = read_msp_response(ser)
            channels = parse_msp_rc(response)
            
            if channels:
                print("RC Channels:", channels)
            else:
                print("No data received.")
            
            time.sleep(0.05)  # small loop delay
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()

