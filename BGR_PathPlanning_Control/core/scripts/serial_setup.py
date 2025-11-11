import serial.tools.list_ports
from icecream import ic
import subprocess
import os
from time import sleep


#GPS_IMU port info
# ic| port.__dict__: {'description': 'CANable 9fddea4 github.com/normaldotcom/canable-fw.git',
#                     'device': '/dev/ttyACM0',
#                     'device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-5/3-5:1.0',
#                     'hwid': 'USB VID:PID=AD50:60C4 SER=003400225734570420393235 LOCATION=3-5:1.0',
#                     'interface': None,
#                     'location': '3-5:1.0',
#                     'manufacturer': 'Protofusion Labs',
#                     'name': 'ttyACM0',
#                     'pid': 24772,
#                     'product': 'CANable 9fddea4 github.com/normaldotcom/canable-fw.git',
#                     'serial_number': '003400225734570420393235',
#                     'subsystem': 'usb',
#                     'usb_device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-5',
#                     'usb_interface_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-5/3-5:1.0',
#                     'vid': 44368}

# Steering port info
# ic| port.__dict__: {'description': 'STM32 STLink - ST-Link VCP Ctrl',
#                     'device': '/dev/ttyACM0',
#                     'device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.1/3-1.1:1.2',
#                     'hwid': 'USB VID:PID=0483:374B SER=066FFF485482494867052116 '
#                             'LOCATION=3-1.1:1.2',
#                     'interface': 'ST-Link VCP Ctrl',
#                     'location': '3-1.1:1.2',
#                     'manufacturer': 'STMicroelectronics',
#                     'name': 'ttyACM0',
#                     'pid': 14155,
#                     'product': 'STM32 STLink',
#                     'serial_number': '066FFF485482494867052116',
#                     'subsystem': 'usb',
#                     'usb_device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.1',
#                     'usb_interface_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.1/3-1.1:1.2',
#                     'vid': 1155}

# torque port info
# ic| port.__dict__: {'description': 'STM32 STLink - ST-Link VCP Ctrl',
#                     'device': '/dev/ttyACM1',
#                     'device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.2/3-1.2:1.2',
#                     'hwid': 'USB VID:PID=0483:374B SER=0667FF555071494867081814 '
#                             'LOCATION=3-1.2:1.2',
#                     'interface': 'ST-Link VCP Ctrl',
#                     'location': '3-1.2:1.2',
#                     'manufacturer': 'STMicroelectronics',
#                     'name': 'ttyACM1',
#                     'pid': 14155,
#                     'product': 'STM32 STLink',
#                     'serial_number': '0667FF555071494867081814',
#                     'subsystem': 'usb',
#                     'usb_device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.2',
#                     'usb_interface_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-1/3-1.2/3-1.2:1.2',
#                     'vid': 1155}
# ic| port.__dict__: {'description': 'STM32 STLink - ST-Link VCP Ctrl',
#                     'device': '/dev/ttyACM0',
#                     'device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-2/3-2.1/3-2.1:1.2',
#                     'hwid': 'USB VID:PID=0483:374B SER=0671FF363154413043225624 '
#                             'LOCATION=3-2.1:1.2',
#                     'interface': 'ST-Link VCP Ctrl',
#                     'location': '3-2.1:1.2',
#                     'manufacturer': 'STMicroelectronics',
#                     'name': 'ttyACM0',
#                     'pid': 14155,
#                     'product': 'STM32 STLink',
#                     'serial_number': '0671FF363154413043225624',
#                     'subsystem': 'usb',
#                     'usb_device_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-2/3-2.1',
#                     'usb_interface_path': '/sys/devices/pci0000:00/0000:00:14.0/usb3/3-2/3-2.1/3-2.1:1.2',
#                     'vid': 1155}

KNOWN_DEVICES = {
    "GPS_IMU": {
        "serial_number": "003400225734570420393235",
    },
    "steering": {
        "serial_number": "0671FF363154413043225624",
    },
    "torque": {
        "serial_number": "0667FF555071494867081814",
    }
}

def can_interface_exists(interface="can0"):
    try:
        result = subprocess.run(["ip", "link", "show", interface], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        return result.returncode == 0
    except Exception as e:
        print(f"Error checking CAN interface: {e}")
        return False

def setup_can_interface(serial_device, can_interface="can0"):
    print(f"Setting up CAN interface '{can_interface}' on {serial_device}...")

    try:
        # Run slcand
        subprocess.run(["sudo", "slcand", "-o", "-c", "-s8", serial_device, can_interface], check=True)
        sleep(1)
        # Bring interface up
        subprocess.run(["sudo", "ip", "link", "set", can_interface, "up"], check=True)
        print(f"CAN interface '{can_interface}' is now up and running.")
    except subprocess.CalledProcessError as e:
        print(f"Failed to set up CAN interface: {e}")

def undo_can_setup(can_interface="can0"):
    try:
        print(f"Bringing down {can_interface}...")
        subprocess.run(["sudo", "ip", "link", "set", can_interface, "down"], check=True)
        print(f"Killing slcand...")
        subprocess.run(["sudo", "pkill", "slcand"], check=True)
        print("CAN interface shut down and slcand stopped.")
    except subprocess.CalledProcessError as e:
        print(f"Error during undo: {e}")

def get_devices():
    ports = serial.tools.list_ports.comports()
    devices = {}
    for port in ports:
        for name, props in KNOWN_DEVICES.items():
            if (port.serial_number == props["serial_number"]):
                devices[name] = port.device
                if name == "GPS_IMU":
                    undo_can_setup("can0")
                    setup_can_interface(port.device, "can0")
    return devices

def main():
    ports = serial.tools.list_ports.comports()
    result = {}
    for port in ports:
        for name, props in KNOWN_DEVICES.items():
            if (port.serial_number == props["serial_number"]):
                result[name] = port.device
                if name == "GPS_IMU":
                    undo_can_setup("can0")
                    setup_can_interface(port.device, "can0")
    ic(result)
if __name__ == '__main__':
    main()