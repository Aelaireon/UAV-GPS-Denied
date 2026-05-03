import tfmpi2c as tfm

# Initialize I2C port (usually 1 on Raspberry Pi) and default address 0x10
if tfm.begin(1, 0x10):
    print("I2C Communication Initialized")
    
    while True:
        # Request a new data frame from the sensor
        if tfm.getData():
            print(f"Dist: {tfm.dist}cm | Flux: {tfm.flux} | Temp: {tfm.temp}°C")
        else:
            tfm.printStatus() # Prints error codes if data read fails
