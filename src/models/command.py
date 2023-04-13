import functools
import numpy as np
import itertools
import driver.g2s_defs as g2s_defs

MIN_FRAME_SIZE = 8  # Header + Address + Command byte + Data length

class Command:
    def __init__(self, command_type, address=0x00, data_length=0, data=bytearray()):
        self._command_type = command_type
        self._address = address
        self._data_length = data_length
        self._data = data
        
    @property
    def command_type(self):
        return self._command_type
    
    @property
    def address(self):
        return self._address
    
    @property 
    def data_length(self):
        return self._data_length
    
    @property
    def data(self):
        return self._data    
    
    @classmethod
    def from_frame(cls, frame):
        # If the frame is too short to extract the data length
        if len(frame) < MIN_FRAME_SIZE:
            return None
        
        # Check if it is the start command
        if not np.array_equal(frame[:len(g2s_defs.PACKET_HEADER)], g2s_defs.PACKET_HEADER):
            print("No header found")
            return None

        # Get data length from frame
        data_length = (frame[7] << 8) | (frame[6] & 0xFF)
        end_of_frame = 8 + data_length + 1  # +1 stand for the checksum
        
        # If the data length exceed the buffer, return None
        if end_of_frame > len(frame):
            return None

        # If the checksum is not 0, return
        # if cls._compute_checksum(frame[:end_of_frame - 1]) ^ frame[end_of_frame - 1]:
        #     print("Bad checksum")
        #     return None

        # Return the command
        return (Command(command_type=frame[5],
                       address=frame[4],
                       data_length = data_length,
                       data=np.array(np.frombuffer(frame[8:8 + data_length], dtype=np.uint8))),
                end_of_frame)
    
    
    def get_bytes(self):
        data_length = ([len(self._data) & 0xFF, (len(self._data) & 0xFF00) >> 8])
        frame_bytes = list(g2s_defs.PACKET_HEADER) + [self._address, self._command_type] + data_length + list(self._data)
        frame_bytes.append(self._compute_checksum(frame_bytes))
        
        return bytearray(frame_bytes)

    @classmethod
    def _compute_checksum(cls, bytes):
        return functools.reduce(lambda acc, val: acc + val, bytes[len(g2s_defs.PACKET_HEADER):]) % 256
    
    def __str__(self):
        string = "--- Command ---\n"
        string += "Address: {}\n".format(hex(self._address))
        string += "Command type: {}\n".format(hex(self._command_type))
        string += "Data: {}\n".format(self._data)
        
        return string