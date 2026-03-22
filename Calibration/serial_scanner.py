import serial.tools.list_ports

def scan_ports():
    print("Scanning for ST-LINK Motor Controllers...\n")
    ports = serial.tools.list_ports.comports()
    
    found_motors = 0
    for port in ports:
        # The B-G431B-ESC1 usually identifies as an STMicroelectronics STLink Virtual COM Port
        if "STLink" in port.description or "STMicroelectronics" in port.manufacturer:
            print(f"Port: {port.device}")
            print(f"  Description: {port.description}")
            print(f"  Unique Serial ID: {port.serial_number}")
            print("-" * 40)
            found_motors += 1
            
    print(f"Found {found_motors} potential motor controllers.")

if __name__ == "__main__":
    scan_ports()