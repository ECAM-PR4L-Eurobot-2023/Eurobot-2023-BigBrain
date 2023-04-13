import numpy as np

from driver.g2s_defs import VERSION_INFO, VERSION_LENGTH, SERIAL_NUMBER_LENGTH
from models.command_data_models.command_data import CommandData


class VersionInformation(CommandData, object):
    def __init__(self, version, serial_number):
        super(VersionInformation, self).__init__()
        self._version = version
        self._serial_number = serial_number
        
    @classmethod
    def from_command(cls, command):
        # Check if it is the correct command code
        if command.command_type != VERSION_INFO:
            return None
        
        # Extract the version and serial number
        return VersionInformation(
            version = '.'.join(np.char.mod("%d", np.flip(command.data[:VERSION_LENGTH]))),
            serial_number=''.join(np.char.mod("%d", np.flip(
                command.data[VERSION_LENGTH:VERSION_LENGTH + SERIAL_NUMBER_LENGTH])))
        )
        
    def __str__(self):
        string = "--- Version Information ---\n"
        string += "Version: {} - Serial Number: {}\n".format(self._version, self._serial_number)
        
        return string