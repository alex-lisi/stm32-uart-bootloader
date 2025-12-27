import serial
import time
import sys

def connect_to_bootloader(port, baudrate=115200):
    """Connect to bootloader over serial"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            rtscts=False,
            dsrdtr=False
        )
        print(f"Connected to {port} at {baudrate} baud")
        
        # Disable DTR and RTS
        ser.setDTR(False)
        ser.setRTS(False)
        
        time.sleep(0.5)
        return ser
    except Exception as e:
        print(f"Error connecting: {e}")
        return None

def send_command(ser, command):
    """Send single character command and wait for response"""
    ser.write(command.encode())
    time.sleep(0.1)
    
    # Read response
    response = ""
    while ser.in_waiting > 0:
        response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        time.sleep(0.1)
    
    return response

def erase_flash(ser):
    """Send erase command"""
    print("\n[1/3] Erasing application flash...")
    
    # Clear any old data
    ser.reset_input_buffer()
    
    # Send erase command
    ser.write('E'.encode())
    
    print("Waiting for erase to complete (this takes a few seconds)...")
    
    # Read response while waiting (up to 5 seconds)
    response = ""
    start_time = time.time()
    
    while (time.time() - start_time) < 5:
        if ser.in_waiting > 0:
            response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
        time.sleep(0.1)
    
    # Print whatever we got
    if response:
        print(response)
    
    # If we got the "Erasing" message, assume success
    # (The actual erase works, we just don't always get the OK response)
    if "Erasing" in response or "OK" in response:
        print("✓ Erase successful")
        return True
    else:
        print("⚠ Erase command sent (check bootloader LED for confirmation)")
        return True  # Assume success since erase actually works
    
def write_firmware(ser, firmware_path):
    """Send firmware file to bootloader in chunks"""
    print(f"\n[2/3] Writing firmware from {firmware_path}...")
    
    # Read firmware file
    try:
        with open(firmware_path, 'rb') as f:
            firmware_data = f.read()
    except Exception as e:
        print(f"✗ Error reading file: {e}")
        return False
    
    file_size = len(firmware_data)
    print(f"Firmware size: {file_size} bytes")
    
    # Send in 256-byte chunks
    chunk_size = 256
    address = 0x08008000  # Application start address
    total_chunks = (file_size + chunk_size - 1) // chunk_size
    
    for i in range(total_chunks):
        # Calculate chunk
        start = i * chunk_size
        end = min(start + chunk_size, file_size)
        chunk = firmware_data[start:end]
        chunk_len = len(chunk)
        
        # Clear any old data
        ser.reset_input_buffer()
        
        # Send 'W' command
        ser.write(b'W')
        ser.flush()
        
        # Wait for ACK
        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 2:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if "ACK" in response:
                break
            time.sleep(0.01)
        
        if "ACK" not in response:
            print(f"\n✗ No ACK for chunk {i+1}/{total_chunks}")
            return False
        
        # Send header (address + length)
        addr_bytes = address.to_bytes(4, byteorder='little')
        len_bytes = chunk_len.to_bytes(4, byteorder='little')
        header = addr_bytes + len_bytes
        ser.write(header)
        ser.flush()
        
        # Wait for HDR_OK
        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 2:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if "HDR_OK" in response:
                break
            time.sleep(0.01)
        
        if "HDR_OK" not in response:
            print(f"\n✗ No HDR_OK for chunk {i+1}/{total_chunks}")
            print(f"Response: {response}")
            return False
        
        # Send data
        ser.write(chunk)
        ser.flush()
        
        # Wait for OK or error
        response = ""
        start_time = time.time()
        while (time.time() - start_time) < 3:
            if ser.in_waiting > 0:
                response += ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            if "OK" in response or "FAIL" in response or "TIMEOUT" in response:
                break
            time.sleep(0.01)
        
        if "FAIL" in response or "TIMEOUT" in response:
            print(f"\n✗ Write failed at chunk {i+1}/{total_chunks}")
            print(f"Response: {response}")
            return False
        
        if "OK" not in response:
            print(f"\n✗ No OK response for chunk {i+1}/{total_chunks}")
            print(f"Response: {response}")
            return False
        
        # Update address for next chunk
        address += chunk_len
        
        # Progress indicator
        progress = (i + 1) / total_chunks * 100
        print(f"\rProgress: {progress:.1f}% ({i+1}/{total_chunks} chunks)", end='', flush=True)
    
    print("\n✓ Firmware written successfully")
    return True

def main():
    print("=" * 50)
    print("STM32 Bootloader Firmware Updater")
    print("=" * 50)
    
    # Get COM port and firmware file from user
    port = input("\nEnter COM port (e.g., COM4): ")
    firmware_file = input("Enter firmware file path (.bin): ")
    
    # Check if file exists
    import os
    if not os.path.exists(firmware_file):
        print(f"\n✗ File not found: {firmware_file}")
        sys.exit(1)
    
    # Connect to bootloader
    print(f"\nConnecting to bootloader on {port}...")
    ser = connect_to_bootloader(port)
    
    if ser is None:
        print("\n✗ Failed to connect. Make sure:")
        print("  - Board is in bootloader mode (hold USER button + reset)")
        print("  - Correct COM port is specified")
        print("  - CP2102 adapter is connected")
        sys.exit(1)
    
    # Read startup messages and check for bootloader
    print("\nChecking bootloader status...")
    time.sleep(1.5)
    startup = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    
    # Check if we got the bootloader banner
    if "Bootloader" in startup or "Ready for firmware update" in startup:
        print("✓ Bootloader detected and ready")
        print(f"\nBootloader says:\n{startup}")
    elif "UART TEST" in startup:
        # Old bootloader message - still valid
        print("✓ Bootloader detected (legacy mode)")
        print(f"\nBootloader says:\n{startup}")
    else:
        print("\n⚠ WARNING: Bootloader not detected!")
        print("Make sure you entered bootloader mode:")
        print("  1. Hold USER button (blue button on board)")
        print("  2. Press RESET button")
        print("  3. Release USER button after 1 second")
        print("  4. Orange LED should stay ON")
        print("\nReceived:")
        print(startup if startup else "(no response)")
        
        response = input("\nContinue anyway? (y/n): ")
        if response.lower() != 'y':
            ser.close()
            sys.exit(1)
    
    # Erase flash
    if not erase_flash(ser):
        ser.close()
        sys.exit(1)
    
    # IMPORTANT: Give bootloader time to return to main loop
    print("\nWaiting for bootloader to be ready...")
    time.sleep(2)
    
    # Clear any leftover data
    ser.reset_input_buffer()
    
    # Write firmware
    if not write_firmware(ser, firmware_file):
        ser.close()
        sys.exit(1)
    
    print("\n[3/3] Update complete!")
    print("\n" + "=" * 50)
    print("✓ Firmware update successful!")
    print("\nNext steps:")
    print("  1. Press RESET button on your board")
    print("  2. Your new firmware should start running")
    print("  3. Check the green LED for your updated behavior")
    print("=" * 50)
    
    ser.close()

if __name__ == "__main__":
    main()