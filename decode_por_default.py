import sys

lookupTable = [
    "output port 0",
    "output port 1",
    "output port 2",
    "output port 3",
    "output port 4",
    "output port 5",
    "output port 6",
    "output port 7", "\n", 
   
    "interrupt mask port 0",
    "interrupt mask port 1",
    "interrupt mask port 2",
    "interrupt mask port 3",
    "interrupt mask port 4",
    "interrupt mask port 5",
    "interrupt mask port 6",
    "interrupt mask port 7","\n", 

    "select PWM port 0",
    "select PWM port 1",
    "select PWM port 2",
    "select PWM port 3",
    "select PWM port 4",
    "select PWM port 5",
    "select PWM port 6",
    "select PWM port 7","\n", 

    "inversion port 0",
    "inversion port 1",
    "inversion port 2",
    "inversion port 3",
    "inversion port 4",
    "inversion port 5",
    "inversion port 6",
    "inversion port 7","\n", 

    "pin direction port 0",
    "pin direction port 1",
    "pin direction port 2",
    "pin direction port 3",
    "pin direction port 4",
    "pin direction port 5",
    "pin direction port 6",
    "pin direction port 7","\n", 

    "drive mode - port 0 - Resistive pull-up",
    "drive mode - port 0 - Resistive pull-down",
    "drive mode - port 0 - Open drain high",
    "drive mode - port 0 - Open drain low",
    "drive mode - port 0 - Strong drive",
    "drive mode - port 0 - Slow strong drive",
    "drive mode - port 0 - High impedance","\n", 

    "drive mode - port 1 - Resistive pull-up",
    "drive mode - port 1 - Resistive pull-down",
    "drive mode - port 1 - Open drain high",
    "drive mode - port 1 - Open drain low",
    "drive mode - port 1 - Strong drive",
    "drive mode - port 1 - Slow strong drive",
    "drive mode - port 1 - High impedance","\n", 

    "drive mode - port 2 - Resistive pull-up",
    "drive mode - port 2 - Resistive pull-down",
    "drive mode - port 2 - Open drain high",
    "drive mode - port 2 - Open drain low",
    "drive mode - port 2 - Strong drive",
    "drive mode - port 2 - Slow strong drive",
    "drive mode - port 2 - High impedance","\n", 

    "drive mode - port 3 - Resistive pull-up",
    "drive mode - port 3 - Resistive pull-down",
    "drive mode - port 3 - Open drain high",
    "drive mode - port 3 - Open drain low",
    "drive mode - port 3 - Strong drive",
    "drive mode - port 3 - Slow strong drive",
    "drive mode - port 3 - High impedance","\n", 

    "drive mode - port 4 - Resistive pull-up",
    "drive mode - port 4 - Resistive pull-down",
    "drive mode - port 4 - Open drain high",
    "drive mode - port 4 - Open drain low",
    "drive mode - port 4 - Strong drive",
    "drive mode - port 4 - Slow strong drive",
    "drive mode - port 4 - High impedance","\n", 

    "drive mode - port 5 - Resistive pull-up",
    "drive mode - port 5 - Resistive pull-down",
    "drive mode - port 5 - Open drain high",
    "drive mode - port 5 - Open drain low",
    "drive mode - port 5 - Strong drive",
    "drive mode - port 5 - Slow strong drive",
    "drive mode - port 5 - High impedance","\n", 

    "drive mode - port 6 - Resistive pull-up",
    "drive mode - port 6 - Resistive pull-down",
    "drive mode - port 6 - Open drain high",
    "drive mode - port 6 - Open drain low",
    "drive mode - port 6 - Strong drive",
    "drive mode - port 6 - Slow strong drive",
    "drive mode - port 6 - High impedance","\n", 

    "drive mode - port 7 - Resistive pull-up",
    "drive mode - port 7 - Resistive pull-down",
    "drive mode - port 7 - Open drain high",
    "drive mode - port 7 - Open drain low",
    "drive mode - port 7 - Strong drive",
    "drive mode - port 7 - Slow strong drive",
    "drive mode - port 7 - High impedance","\n", 

    "config setting PWM 0",
    "period setting PWM 0",
    "pulse width setting PWM 0","\n", 

    "config setting PWM 1",
    "period setting PWM 1",
    "pulse width setting PWM 1","\n", 

    "config setting PWM 2",
    "period setting PWM 2",
    "pulse width setting PWM 2","\n", 

    "config setting PWM 3",
    "period setting PWM 3",
    "pulse width setting PWM 3","\n", 

    "config setting PWM 4",
    "period setting PWM 4",
    "pulse width setting PWM 4","\n", 

    "config setting PWM 5",
    "period setting PWM 5",
    "pulse width setting PWM 5","\n", 

    "config setting PWM 6",
    "period setting PWM 6",
    "pulse width setting PWM 6","\n", 

    "config setting PWM 7",
    "period setting PWM 7",
    "pulse width setting PWM 7","\n", 

    "config setting PWM 8",
    "period setting PWM 8",
    "pulse width setting PWM 8","\n", 

    "config setting PWM 9",
    "period setting PWM 9",
    "pulse width setting PWM 9","\n", 

    "config setting PWM 10",
    "period setting PWM 10",
    "pulse width setting PWM 10","\n", 

    "config setting PWM 11",
    "period setting PWM 11",
    "pulse width setting PWM 11","\n", 

    "config setting PWM 12",
    "period setting PWM 12",
    "pulse width setting PWM 12","\n", 

    "config setting PWM 13",
    "period setting PWM 13",
    "pulse width setting PWM 13","\n", 

    "config setting PWM 14",
    "period setting PWM 14",
    "pulse width setting PWM 14","\n", 

    "config setting PWM 15",
    "period setting PWM 15",
    "pulse width setting PWM 15","\n", 

    "divider",
    "enable",
    "crc"
]


data = []

data = sys.stdin.readline()
data = data.split(" ")

try:
    data.remove("\n")
    data.remove("\r")
except ValueError:
    pass

idx = 0
for el in lookupTable:
    if(el != "\n"):
        try:
            print(el + " => " + data[idx])
            idx = idx + 1
        except IndexError:
            print(el + " => MISSING!!!")
    else:
        print(el)

