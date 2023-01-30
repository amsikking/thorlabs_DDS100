import time
import serial

class Controller:
    '''
    Basic device adaptor for thorlabs DDS100 compact 100 mm direct drive stage.
    - Uses a KBD101 K-Cube brushless DC servo driver (controller)
    Test code runs and seems robust.
    '''
    def __init__(
        self, which_port, name='DDS100', verbose=True, very_verbose=False):
        self.name = name
        self.verbose = verbose
        self.very_verbose = very_verbose
        if self.verbose: print("%s: opening..."%self.name, end='')
        try:
            self.port = serial.Serial( # extra long timeout needed for ._home()
                port=which_port, baudrate=115200, timeout=15)
        except serial.serialutil.SerialException:
            raise IOError(
                '%s: no connection on port %s'%(self.name, which_port))
        if self.verbose: print(" done.")
        self._get_info()
        assert self.model_number == 'KBD101\x00\x00'
        assert self.firmware_v == 131080
        self.position_min_mm = 0
        self.position_max_mm = 100
        self.range_tol_mm = 0.1 # avoids range check assertion error
        self.EncCnt_per_mm = 2000
        self.offset_mm = 0.05 # avoids encoder count overflow at zero position
        self.offset_counts = int(round(self.offset_mm * self.EncCnt_per_mm))
        self._set_enable(True)
        self._home()
        self.get_position_mm()
        self._moving = False
        self.move_mm(0, relative=False)

    def _send(self, cmd, response_bytes=None):
        if self.very_verbose: print('%s: sending cmd ='%self.name, cmd)
        self.port.write(cmd)
        if response_bytes is not None:
            response = self.port.read(response_bytes)
        else:
            response = None
        assert self.port.inWaiting() == 0
        if self.very_verbose: print('%s: -> response = '%self.name, response)
        return response

    def _get_info(self): # MGMSG_HW_REQ_INFO
        if self.verbose:
            print('%s: getting info'%self.name)
        cmd = b'\x05\x00\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=90)
        self.model_number = response[10:18].decode('ascii')
        self.type = int.from_bytes(response[18:20], byteorder='little')
        self.serial_number = int.from_bytes(response[6:10], byteorder='little')
        self.firmware_v = int.from_bytes(response[20:24], byteorder='little')
        self.hardware_v = int.from_bytes(response[84:86], byteorder='little')
        if self.verbose:
            print('%s: -> model number  = %s'%(self.name, self.model_number))
            print('%s: -> type          = %s'%(self.name, self.type))
            print('%s: -> serial number = %s'%(self.name, self.serial_number))
            print('%s: -> firmware version = %s'%(self.name, self.firmware_v))
            print('%s: -> hardware version = %s'%(self.name, self.hardware_v))
        return response

    def _get_enable(self): # MGMSG_MOD_REQ_CHANENABLESTATE
        if self.verbose:
            print('%s: getting enable'%self.name)
        cmd = b'\x11\x02\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=6)
        assert int(response[3]) in (1, 2)
        if int(response[3]) == 1: self.enable = True
        if int(response[3]) == 2: self.enable = False
        if self.verbose:
            print('%s: -> enable = %s'%(self.name, self.enable))
        return self.enable

    def _set_enable(self, enable): # MGMSG_MOD_SET_CHANENABLESTATE
        assert enable in (True, False)
        if enable:     cmd = b'\x10\x02\x00\x01\x50\x01'
        if not enable: cmd = b'\x10\x02\x00\x02\x50\x01'
        if self.verbose:
            print('%s: setting enable = %s'%(self.name, enable))
        self._send(cmd)
        assert self._get_enable() == enable
        if self.verbose:
            print('%s: done setting enable'%self.name)
        return None

    def _home(self): # MGMSG_MOT_MOVE_HOME
        if self.verbose:
            print('%s: homing stage...'%self.name)
        cmd = b'\x43\x04\x00\x00\x50\x01'
        # response_bytes=6 is not documented, discovered by trial and error
        # when the 6 bytes return it seems like the home routine is finished!
        self._send(cmd, response_bytes=6)
        if self.verbose:
            print('%s: -> done homing stage'%self.name)
        return None

    def identify(self): # MGMSG_MOD_IDENTIFY
        if self.verbose:
            print('%s: -> flashing front panel LEDs'%self.name)
        cmd = b'\x23\x02\x00\x00\x50\x01'
        response = self._send(cmd)
        return response

    def get_position_mm(self): # MGMSG_MOT_REQ_POSCOUNTER
        if self.verbose:
            print('%s: getting position'%self.name)
        cmd = b'\x11\x04\x00\x00\x50\x01'
        response = self._send(cmd, response_bytes=12)
        self.ch_id_bytes = response[6:8]
        position_counts = int.from_bytes(response[8:12], byteorder='little')
        position_mm = position_counts / self.EncCnt_per_mm
        self.position_mm = position_mm - self.offset_mm # shift away from zero
        if self.verbose:
            print('%s: -> position = %0.4fmm'%(self.name, self.position_mm))
        return self.position_mm

    def _finish_move(self, polling_wait_s=0.01):
        if not self._moving: return
        self.port.read(20) # collect bytes to confirm move (could parse too...)
        self._moving = False
        if self.verbose:
            print('%s: -> finished moving'%(self.name))
        return None

    def move_mm(self, move_mm, relative=True, block=True):
        if self._moving: self._finish_move()
        position_counts = int(round(move_mm * self.EncCnt_per_mm)) # integer
        move_mm = position_counts / self.EncCnt_per_mm # legalize move
        if self.verbose:
            print('%s: moving mm = %0.4fmm (relative=%s)'%(
                self.name, move_mm, relative))
        d = bytes([b'\x50'[0] | b'\x80'[0]]) # 'destination byte'
        if relative:        # MGMSG_MOT_MOVE_RELATIVE
            self.position_mm = self.position_mm + move_mm
            p = position_counts.to_bytes(4, byteorder='little', signed=True)
            cmd = (b'\x48\x04\x06\x00' + d + b'\x01' + self.ch_id_bytes + p)
        if not relative:    # MGMSG_MOT_MOVE_ABSOLUTE
            self.position_mm = move_mm
            position_counts = position_counts + self.offset_counts
            p = position_counts.to_bytes(4, byteorder='little', signed=True)
            cmd = (b'\x53\x04\x06\x00' + d + b'\x01' + self.ch_id_bytes + p)
        assert self.position_mm >= self.position_min_mm - self.range_tol_mm
        assert self.position_mm <= self.position_max_mm + self.range_tol_mm
        self._send(cmd)
        self._moving = True
        if block:
            self._finish_move()
        return None

    def close(self):
        if self.verbose: print("%s: closing..."%self.name, end=' ')
        self.port.close()
        if self.verbose: print("done.")
        return None

if __name__ == '__main__':
    stage = Controller('COM3', verbose=True, very_verbose=False)

##    stage.identify()

    print('\n# Get position:')
    stage.get_position_mm()

    print('\n# Test max range:')
    stage.move_mm(100, relative=False)
    stage.move_mm(0, relative=False)

    print('\n# Some relative moves:')
    for moves in range(3):
        move = stage.move_mm(20)
    for moves in range(3):
        move = stage.move_mm(-20)

    print('\n# Some random absolute moves:')
    from random import uniform
    for moves in range(3):
        random_move_mm = uniform(0, 100)
        move = stage.move_mm(random_move_mm, relative=False)

    print('\n# Non-blocking move:')
    stage.move_mm(20, relative=False, block=False)
    stage.move_mm( 0, relative=False, block=False)
    print('(immediate follow up call forces finish on pending move)')
    print('doing something else')
    stage._finish_move()

    stage.close()
