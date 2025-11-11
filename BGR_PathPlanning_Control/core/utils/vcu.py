import serial 
import serial.tools.list_ports
import time
import struct
import threading


class VCU:
    def __init__(self, port_name = "", id = 0x01):
        assert isinstance(port_name, str), 'Port name must be string'
        assert isinstance(id, int), 'ID must be integer'
        self._port = self._open_port(port_name)
        self._debug_thread = None
        self._angle_setpoint = 0
        self._torque_setpoint = 0
        self._id = id
        self._is_debug = False
        self._last_write_time = time.time()
        self._write_delay = 0.001
        self._last_psoition = 0
        self._is_closing = False


    def set_angle(self, angle):
        assert isinstance(angle, float), 'Angle must be float'
        last_psoition = self._last_psoition
        self._last_psoition = self._angle_setpoint
        self._angle_setpoint = angle #- last_psoition
        self._write_to_port()
    
    def set_torque(self, torque):
        assert isinstance(torque, float), 'Torque must be float'
        self._torque_setpoint = torque
        self._write_to_port()

    def _write_to_port(self):
        if time.time() - self._last_write_time < self._write_delay:
            return
        self._last_write_time = time.time()
        data = struct.pack('<hff',self._id, self._angle_setpoint, self._torque_setpoint)
        try:
            self._port.write(data)
            if self._is_debug:
                pass
                # print(f"sending: id = {self._id}, angle = {self._angle_setpoint}, torque = {self._torque_setpoint}")
        except serial.SerialException:
            print("Could not write to port")

    def _read_from_port(self):
        if not self._port or not self._port.is_open:
            return None
        reading = self._port.readline().decode('utf-8').strip()
        if reading:
                return reading

    def debug(self):
        self._is_debug = True
        self._debug_thread = threading.Thread(target=self._debug_print)
        self._debug_thread.start()

    def _open_port(self, port_name):
        try:
            port = serial.Serial(port_name, 115200)
            if port is None or not port.is_open:
                raise serial.SerialException
            return port
        except serial.SerialException:
            print("Could not open port")
            print("Select port from the following list:")
            ports = serial.tools.list_ports.comports()
            for i, port in enumerate(ports):
                print(f"{i}. " + port.device)
            port = input("port number: ")
            port = self._open_port(ports[int(port)].device)
            return port 

    def _debug_print(self):
        while True:
            string = self._read_from_port()
            if string:
                print(string)

    def __del__(self):
        print("closing .. ")
        if self._port and self._port.is_open:
            self._port.close()
        if self._debug_thread:
            self._debug_thread.join()
        
        

    def close(self):
        if not self._is_closing:  # Prevent multiple closes
            self._is_closing = True
            try:
                # Set safe values before closing
                self.set_angle(0.0)
                self.set_torque(0.0)
                time.sleep(0.1)  # Small delay to ensure commands are sent
                
                if self._port and self._port.is_open:
                    self._port.close()
                if self._debug_thread:
                    self._debug_thread.join(timeout=1.0)  # Add timeout to prevent hanging
            except Exception as e:
                print(f"Error during VCU cleanup: {str(e)}")

    def __del__(self):
        self.close()

def cleanup_vcus(vcu_steering = None, vcu_tourqe = None):
    print("Cleaning up VCU connections...")
    try:
        # Set safe values before closing
        if vcu_steering is not None:
            vcu_steering.set_angle(0.0)
        if vcu_tourqe is not None:
            vcu_tourqe.set_torque(0.0)
        time.sleep(1)  # Small delay to ensure commands are sent
        
        # Close the VCU connections
        if vcu_steering is not None:
            vcu_steering.close()
        if vcu_tourqe is not None:
            vcu_tourqe.close()
        print("VCU connections closed successfully")
    except Exception as e:
        print(f"Error during cleanup: {str(e)}")