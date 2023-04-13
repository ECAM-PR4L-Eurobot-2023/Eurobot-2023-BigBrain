""" Module that contains the frame builder used to creat the frame for the GS2 """

import driver.g2s_defs as g2s_defs
from models.command import Command

class CommandBuilder:
    @classmethod
    def get_device_address(cls):
        return Command(g2s_defs.DEVICE_ADDRESS).get_bytes()
    
    @classmethod
    def get_version_info(cls):
        return Command(g2s_defs.VERSION_INFO).get_bytes()
    
    @classmethod
    def get_device_parameters(cls):
        return Command(g2s_defs.DEVICE_PARAMETERS).get_bytes()
    
    @classmethod
    def start_scan(cls):
        return Command(g2s_defs.START_SCAN).get_bytes()
    
    @classmethod
    def stop_scan(cls):
        return Command(g2s_defs.STOP_SCAN).get_bytes()
    
    @classmethod
    def set_baudrate(cls, baudrate):
        print(baudrate)
        print(len([baudrate]))
        return Command(g2s_defs.SET_BAUDRATE, data=[baudrate]).get_bytes()
    
    @classmethod
    def set_edge_mode(cls, edge_mode):
        return Command(g2s_defs.SET_EDGE_MODE, data=bytearray(edge_mode)).get_bytes()

    @classmethod
    def system_reset(cls, address):
        return Command(g2s_defs.RESTART, address=address).get_bytes()
