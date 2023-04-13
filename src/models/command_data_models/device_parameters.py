from driver.g2s_defs import DEVICE_PARAMETERS, K_COMPENSATION, B_COMPENSATION, BIAS_COMPENSATION
from models.command_data_models.command_data import CommandData


class DeviceParameters(CommandData, object):
    def __init__(self, k0, b0, k1, b1, bias):
        super(DeviceParameters, self).__init__()
        self._k0 = k0
        self._b0 = b0
        self._k1 = k1
        self._b1 = b1
        self._bias = bias
        
    @property
    def k0(self):
        return self._k0
    
    @property
    def b0(self):
        return self._b0

    @property
    def k1(self):
        return self._k1

    @property
    def b1(self):
        return self._b1

    @property
    def bias(self):
        return self._bias

    @classmethod
    def from_command(cls, command):
        # If the command type is not [DEVICE_PARAMETERS], return None
        if command.command_type != DEVICE_PARAMETERS:
            return None
        
        # Extract every parameters
        k0 = ((command.data[1] << 8) | (command.data[0] & 0xFF)) / K_COMPENSATION
        b0 = ((command.data[3] << 8) | (command.data[2] & 0xFF)) / B_COMPENSATION
        k1 = ((command.data[5] << 8) | (command.data[4] & 0xFF)) / K_COMPENSATION
        b1 = ((command.data[7] << 8) | (command.data[6] & 0xFF)) / B_COMPENSATION
        bias = command.data[8] / BIAS_COMPENSATION

        return DeviceParameters(k0, b0, k1, b1, bias)
    
    def __str__(self):
        string = "--- Device Parameters ---\n"
        string += "k0: {} - b0: {}\n".format(self._k0, self._b0)
        string += "k1: {} - b1: {}\n".format(self._k1, self._b1)
        string += "bias: {}\n".format(self._bias)
        
        return string