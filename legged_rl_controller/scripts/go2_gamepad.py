import sys

class KeySwitchComponents:
    def __init__(self, value=0):
        self._value = value

    @property
    def R1(self):
        return (self._value >> 0) & 0x01

    @R1.setter
    def R1(self, bit):
        if bit:
            self._value |= (1 << 0)
        else:
            self._value &= ~(1 << 0)

    @property
    def L1(self):
        return (self._value >> 1) & 0x01

    @L1.setter
    def L1(self, bit):
        if bit:
            self._value |= (1 << 1)
        else:
            self._value &= ~(1 << 1)

    @property
    def start(self):
        return (self._value >> 2) & 0x01

    @start.setter
    def start(self, bit):
        if bit:
            self._value |= (1 << 2)
        else:
            self._value &= ~(1 << 2)

    @property
    def select(self):
        return (self._value >> 3) & 0x01

    @select.setter
    def select(self, bit):
        if bit:
            self._value |= (1 << 3)
        else:
            self._value &= ~(1 << 3)

    @property
    def R2(self):
        return (self._value >> 4) & 0x01

    @R2.setter
    def R2(self, bit):
        if bit:
            self._value |= (1 << 4)
        else:
            self._value &= ~(1 << 4)

    @property
    def L2(self):
        return (self._value >> 5) & 0x01

    @L2.setter
    def L2(self, bit):
        if bit:
            self._value |= (1 << 5)
        else:
            self._value &= ~(1 << 5)

    @property
    def F1(self):
        return (self._value >> 6) & 0x01

    @F1.setter
    def F1(self, bit):
        if bit:
            self._value |= (1 << 6)
        else:
            self._value &= ~(1 << 6)

    @property
    def F2(self):
        return (self._value >> 7) & 0x01

    @F2.setter
    def F2(self, bit):
        if bit:
            self._value |= (1 << 7)
        else:
            self._value &= ~(1 << 7)

    @property
    def A(self):
        return (self._value >> 8) & 0x01

    @A.setter
    def A(self, bit):
        if bit:
            self._value |= (1 << 8)
        else:
            self._value &= ~(1 << 8)

    @property
    def B(self):
        return (self._value >> 9) & 0x01

    @B.setter
    def B(self, bit):
        if bit:
            self._value |= (1 << 9)
        else:
            self._value &= ~(1 << 9)

    @property
    def X(self):
        return (self._value >> 10) & 0x01

    @X.setter
    def X(self, bit):
        if bit:
            self._value |= (1 << 10)
        else:
            self._value &= ~(1 << 10)

    @property
    def Y(self):
        return (self._value >> 11) & 0x01

    @Y.setter
    def Y(self, bit):
        if bit:
            self._value |= (1 << 11)
        else:
            self._value &= ~(1 << 11)

    @property
    def up(self):
        return (self._value >> 12) & 0x01

    @up.setter
    def up(self, bit):
        if bit:
            self._value |= (1 << 12)
        else:
            self._value &= ~(1 << 12)

    @property
    def right(self):
        return (self._value >> 13) & 0x01

    @right.setter
    def right(self, bit):
        if bit:
            self._value |= (1 << 13)
        else:
            self._value &= ~(1 << 13)

    @property
    def down(self):
        return (self._value >> 14) & 0x01

    @down.setter
    def down(self, bit):
        if bit:
            self._value |= (1 << 14)
        else:
            self._value &= ~(1 << 14)

    @property
    def left(self):
        return (self._value >> 15) & 0x01

    @left.setter
    def left(self, bit):
        if bit:
            self._value |= (1 << 15)
        else:
            self._value &= ~(1 << 15)

    def get_value(self):
        return self._value

    def set_value(self, val):
        self._value = val
                

class Button:
    def __init__(self):
        self.pressed = False
        self.on_press = False
        self.on_release = False

    def update(self, state: bool):
        self.on_press = (state != self.pressed) if state else False
        self.on_release = False if state else (state != self.pressed)
        self.pressed = state

class Go2Gamepad:
    def __init__(self):
        self.keyswitch = KeySwitchComponents()
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        self.ry = 0.0

        self.smooth = 0.03
        self.dead_zone = 0.01
        
        self.R1 = Button()
        self.L1 = Button()
        self.start = Button()
        self.select = Button()
        self.R2 = Button()
        self.L2 = Button()
        self.F1 = Button()
        self.F2 = Button()
        self.A = Button()
        self.B = Button()
        self.X = Button()
        self.Y = Button()
        self.up = Button()
        self.right = Button()
        self.down = Button()
        self.left = Button()
        
        print("\n\n\n\n")
        
    def update(self, keyswitch: int, lx: float, ly: float, rx: float, ry: float):
        self.keyswitch.set_value(keyswitch)
        # self.lx = self.smooth * (0.0 if abs(lx)<self.dead_zone else lx) + (1 - self.smooth) * self.lx
        # self.ly = self.smooth * (0.0 if abs(ly)<self.dead_zone else ly) + (1 - self.smooth) * self.ly
        # self.rx = self.smooth * (0.0 if abs(rx)<self.dead_zone else rx) + (1 - self.smooth) * self.rx
        # self.ry = self.smooth * (0.0 if abs(ry)<self.dead_zone else ry) + (1 - self.smooth) * self.ry

        self.lx = lx if abs(lx) > self.dead_zone else 0.0
        self.ly = ly if abs(ly) > self.dead_zone else 0.0
        self.rx = rx if abs(rx) > self.dead_zone else 0.0
        self.ry = ry if abs(ry) > self.dead_zone else 0.0
        
        self.R1.update(self.keyswitch.R1)
        self.L1.update(self.keyswitch.L1)
        self.start.update(self.keyswitch.start)
        self.select.update(self.keyswitch.select)
        self.R2.update(self.keyswitch.R2)
        self.L2.update(self.keyswitch.L2)
        self.F1.update(self.keyswitch.F1)
        self.F2.update(self.keyswitch.F2)
        self.A.update(self.keyswitch.A)
        self.B.update(self.keyswitch.B)
        self.X.update(self.keyswitch.X)
        self.Y.update(self.keyswitch.Y)
        self.up.update(self.keyswitch.up)
        self.right.update(self.keyswitch.right)
        self.down.update(self.keyswitch.down)
        self.left.update(self.keyswitch.left)
        
        sys.stdout.write(f"\033[F\033[F\033[F\033[F")
        print("lx\tly\trx\try")
        sys.stdout.write(f"\033[K") # Clear to the end of line
        print(f"{self.lx:.3f}\t{self.ly:.3f}\t{self.rx:.3f}\t{self.ry:.3f}")
        print("R1\tL1\tstart\tselect\tR2\tL2\tF1\tF2\tA\tB\tX\tY\tup\tright\tdown\tleft")
        sys.stdout.write(f"\033[K") # Clear to the end of line
        print(f"{self.R1.pressed}\t{self.L1.pressed}\t{self.start.pressed}\t{self.select.pressed}\t{self.R2.pressed}\t{self.L2.pressed}\t{self.F1.pressed}\t{self.F2.pressed}\t{self.A.pressed}\t{self.B.pressed}\t{self.X.pressed}\t{self.Y.pressed}\t{self.up.pressed}\t{self.right.pressed}\t{self.down.pressed}\t{self.left.pressed}")
        sys.stdout.flush()
        
        
if __name__ == '__main__':
    joy = Go2Gamepad()
    joy.update(0xffff, 0.1, 0.2, 0.3, 0.4)
    