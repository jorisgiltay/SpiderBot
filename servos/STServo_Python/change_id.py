#!/usr/bin/env python
#
# *********     Change Servo ID Example      *********
#
# This script changes a servo's ID from the current ID to a new ID
# Available STServo model on this example : All models using Protocol STS
# This example is tested with a STServo and an URT
#

import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from STservo_sdk import *                 # Uses STServo SDK library

# Configuration - MODIFY THESE VALUES AS NEEDED
CURRENT_ID = 1                      # Current servo ID (default is usually 1)
NEW_ID = 2                          # New servo ID you want to set
BAUDRATE = 1000000                  # STServo default baudrate : 1000000
DEVICENAME = 'COM10'                # Check which port is being used on your controller
                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
                                    
# NOTE: Update DEVICENAME to match your system's COM port

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = sts(portHandler)

def change_servo_id(current_id, new_id):
    """
    Change servo ID from current_id to new_id
    
    Args:
        current_id: The current ID of the servo
        new_id: The new ID to assign to the servo
    
    Returns:
        bool: True if successful, False otherwise
    """
    
    # Open port
    if not portHandler.openPort():
        print("Failed to open the port")
        return False
    
    print(f"Succeeded to open the port {DEVICENAME}")
    
    # Set port baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to change the baudrate")
        portHandler.closePort()
        return False
    
    print(f"Succeeded to change the baudrate to {BAUDRATE}")
    
    # First, ping the servo with current ID to make sure it's connected
    print(f"\nPinging servo with ID {current_id}...")
    model_number, comm_result, error = packetHandler.ping(current_id)
    
    if comm_result != COMM_SUCCESS:
        print(f"Failed to ping servo with ID {current_id}")
        print(f"Error: {packetHandler.getTxRxResult(comm_result)}")
        portHandler.closePort()
        return False
    
    if error != 0:
        print(f"Servo error: {packetHandler.getRxPacketError(error)}")
        portHandler.closePort()
        return False
    
    print(f"Successfully found servo with ID {current_id}, model number: {model_number}")
    
    # Check if new ID is already in use
    print(f"\nChecking if ID {new_id} is already in use...")
    test_model, test_comm_result, test_error = packetHandler.ping(new_id)
    
    if test_comm_result == COMM_SUCCESS:
        print(f"WARNING: A servo with ID {new_id} already exists!")
        print("Please choose a different ID or disconnect the other servo.")
        portHandler.closePort()
        return False
    
    print(f"ID {new_id} is available.")
    
    # Change the servo ID
    print(f"\nChanging servo ID from {current_id} to {new_id}...")
    
    # Write new ID to the servo's ID register (STS_ID = 5)
    comm_result, error = packetHandler.write1ByteTxRx(current_id, STS_ID, new_id)
    
    if comm_result != COMM_SUCCESS:
        print(f"Failed to write new ID to servo")
        print(f"Error: {packetHandler.getTxRxResult(comm_result)}")
        portHandler.closePort()
        return False
    
    if error != 0:
        print(f"Servo error: {packetHandler.getRxPacketError(error)}")
        portHandler.closePort()
        return False
    
    print(f"Successfully wrote new ID {new_id} to servo")
    
    # Verify the change by pinging with the new ID
    print(f"\nVerifying ID change by pinging servo with new ID {new_id}...")
    verify_model, verify_comm_result, verify_error = packetHandler.ping(new_id)
    
    if verify_comm_result != COMM_SUCCESS:
        print(f"Failed to ping servo with new ID {new_id}")
        print(f"Error: {packetHandler.getTxRxResult(verify_comm_result)}")
        portHandler.closePort()
        return False
    
    if verify_error != 0:
        print(f"Servo error: {packetHandler.getRxPacketError(verify_error)}")
        portHandler.closePort()
        return False
    
    print(f"SUCCESS! Servo ID successfully changed from {current_id} to {new_id}")
    print(f"Model number: {verify_model}")
    
    # Try pinging the old ID to confirm it's no longer responding
    print(f"\nConfirming old ID {current_id} is no longer responding...")
    old_model, old_comm_result, old_error = packetHandler.ping(current_id)
    
    if old_comm_result == COMM_SUCCESS:
        print(f"WARNING: Servo still responds to old ID {current_id}")
        print("The ID change may not have been successful.")
    else:
        print(f"Confirmed: No servo responds to old ID {current_id}")
    
    portHandler.closePort()
    return True

def main():
    print("=" * 60)
    print("         Waveshare ST3215-HS Servo ID Change Tool")
    print("=" * 60)
    print(f"Current servo ID: {CURRENT_ID}")
    print(f"New servo ID: {NEW_ID}")
    print(f"Port: {DEVICENAME}")
    print(f"Baudrate: {BAUDRATE}")
    print("=" * 60)
    
    print("\nIMPORTANT NOTES:")
    print("1. Make sure only ONE servo is connected to avoid ID conflicts")
    print("2. The servo must be powered on and properly connected")
    print("3. Make sure the COM port is correct for your system")
    print("4. This will permanently change the servo's ID")
    
    print("\nPress 'y' to continue or any other key to exit...")
    if getch().lower() != 'y':
        print("Operation cancelled.")
        return
    
    # Attempt to change the servo ID
    success = change_servo_id(CURRENT_ID, NEW_ID)
    
    if success:
        print("\n" + "=" * 60)
        print("ID CHANGE COMPLETED SUCCESSFULLY!")
        print(f"Your servo now has ID {NEW_ID}")
        print("You can now use this ID in your other scripts.")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("ID CHANGE FAILED!")
        print("Please check the connection and try again.")
        print("=" * 60)
    
    print("\nPress any key to exit...")
    getch()

if __name__ == "__main__":
    main()
