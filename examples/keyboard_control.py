import picar_4wd as fc
import sys
import tty
import termios
import keyboard_control as kc

power_val = 50
invert_turns = True  # Flag to invert turn directions, True by default

print("If you want to quit, please press q")
print("Press 'i' to toggle turn inversion")

def readchar():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def readkey(getchar_fn=None):
    getchar = getchar_fn or readchar
    c1 = getchar()
    if ord(c1) != 0x1b:
        return c1
    c2 = getchar()
    if ord(c2) != 0x5b:
        return c1
    c3 = getchar()
    return chr(0x10 + ord(c3) - 65)

if __name__ == '__main__':
    kc.Keyboard_control()