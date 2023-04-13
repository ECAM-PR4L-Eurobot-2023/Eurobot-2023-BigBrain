import numpy as np
import serial

from builders.command_buidler import CommandBuilder
import driver.g2s_defs as g2s_defs
from models.command import Command
from models.command_data_models.device_parameters import DeviceParameters
from models.command_data_models.scan_data import ScanData
from models.command_data_models.version_information import VersionInformation


DEFAULT_TIMEOUT = 1


class G2S:
    def __init__(self, port, spatial_parameters, baudrate=g2s_defs.DEFAULT_BAUDRATE, timeout=DEFAULT_TIMEOUT):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._rx_buffer = np.array([], dtype=np.uint8)
        
        # Stock GS2 informations
        self._spatial_parameters = spatial_parameters[:]
        self._version_information = [None] * g2s_defs.MAXIMUM_LIDAR_CHAIN
        self._device_parameters = [None] * g2s_defs.MAXIMUM_LIDAR_CHAIN
        self._scan_data = [None] * g2s_defs.MAXIMUM_LIDAR_CHAIN
        
        # Initialize serial
        self._ser = serial.Serial(port, baudrate, timeout=self._timeout)
        
    @property
    def version_information(self):
        return self._version_information
    
    @property
    def device_parameters(self):
        return self._device_parameters
    
    @property
    def scan_data(self):
        return self._scan_data
        
    def run(self):
        self._continue_reading()
        self._extract_command()
        
    def get_device_address(self):
        self._ser.write(CommandBuilder.get_device_address())
    
    def get_version_information(self):
        self._ser.write(CommandBuilder.get_version_info())
    
    def get_device_parameters(self):
        self._ser.write(CommandBuilder.get_device_parameters())
    
    def start_scan(self):
        self._ser.write(CommandBuilder.start_scan())
    
    def stop_scan(self):
        self._ser.write(CommandBuilder.stop_scan())
    
    def set_baudrate(self, baudrate):
        self._ser.write(CommandBuilder.set_baudrate(np.uint8(baudrate)))
    
    def set_edge_mode(self, edge_mode):
        self._ser.write(CommandBuilder.set_edge_mode(edge_mode))
    
    def system_reset(self):
        self._ser.write(CommandBuilder.system_reset(0x01))
        self._ser.write(CommandBuilder.system_reset(0x02))
        self._ser.write(CommandBuilder.system_reset(0x04))
        self._ser.write(CommandBuilder.system_reset(0x00))
    
    def _continue_reading(self):
        # If data available, read them
        if self._ser.in_waiting > 0:
            self._rx_buffer = np.append(self._rx_buffer, 
                                        np.array(np.frombuffer(self._ser.read_all(),
                                                               dtype=np.uint8)))
            
    def _extract_command(self):
        can_loop = True  # Variable put to false in the case if a command failed

        # start = time.time()
        while np.array_equal(self._rx_buffer[:len(g2s_defs.PACKET_HEADER)], g2s_defs.PACKET_HEADER) and can_loop:
            command_decoder = Command.from_frame(self._rx_buffer)
            
            # If the command has been decoded, delete this frame from the buffer
            if command_decoder is not None:
                command, offset = command_decoder
                
                self._decode(command)
                self._rx_buffer = self._rx_buffer[offset:]
            else:
                can_loop = False

        # Remove useless heading bytes from a previous one
        if not np.array_equal(self._rx_buffer[:len(g2s_defs.PACKET_HEADER)], g2s_defs.PACKET_HEADER) and self._rx_buffer.size:
            for i in range(self._rx_buffer.size):
                if np.array_equal(self._rx_buffer[i:i + len(g2s_defs.PACKET_HEADER)], g2s_defs.PACKET_HEADER[:]):
                    self._rx_buffer = self._rx_buffer[i:]
                    break

    def _decode(self, command):
        index = command.address >> 1

        if command.command_type == g2s_defs.VERSION_INFO:
            self._version_information[index] = VersionInformation.from_command(command)
        elif command.command_type == g2s_defs.DEVICE_PARAMETERS:
            self._device_parameters[index] = DeviceParameters.from_command(command)
            print(self._device_parameters[index])
        elif command.command_type == g2s_defs.START_SCAN:
            self._scan_data[index] = ScanData.from_command(command, self._spatial_parameters[index], self._device_parameters[index])

    def __del__(self):
        self._ser.close()
