import tfmplus as tfm

# Initialize serial port (default baud 115200)
if tfm.begin("/dev/ttyAMA0", 115200):
    # Send the switch command
    if tfm.sendCommand(tfm.SET_I2C_MODE, 0):
        print("Switch to I2C mode successful.")
        # Note: TFMini-Plus switches immediately; 
        # no SAVE_SETTING command is required.
    else:
        tfm.printStatus()
