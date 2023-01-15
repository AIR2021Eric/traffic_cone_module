import serial
import time

ser = serial.Serial('COM4',baudrate=9600,timeout=1)
time.sleep(0.5)

def send_message(order):
    command = ""
    if order == "allON":
        command = b"\xFF\x0F\x00\x00\x00\x08\x01\xFF\x30\x1D"
    if order == "allOFF":
        command = b"\xFF\x0F\x00\x00\x00\x08\x01\x00\x70\x5D"

    if order == "relay1ON":
        command = b"\xFF\x05\x00\x00\xFF\x00\x99\xE4"
    if order == "relay1OFF":
        command = b"\xFF\x05\x00\x00\x00\x00\xD8\x14"

    if order == "relay2ON":
        command = b"\xFF\x05\x00\x01\xFF\x00\xC8\x24"
    if order == "relay2OFF":
        command = b"\xFF\x05\x00\x01\x00\x00\x89\xD4"

    if order == "relay3ON":
        command = b"\xFF\x05\x00\x02\xFF\x00\x38\x24"
    if order == "relay3OFF":
        command = b"\xFF\x05\x00\x02\x00\x00\x79\xD4"

    if order == "relay4ON":
        command = b"\xFF\x05\x00\x03\xFF\x00\x69\xE4"
    if order == "relay4OFF":
        command = b"\xFF\x05\x00\x03\x00\x00\x28\x14"

    ser.write(command)
    print(f'sent {command}')

# send_message("relay2ON")
send_message("relay2OFF")

# for i in range(5):
#     send_message("relay2ON")
#     time.sleep(1)
#     send_message("relay2OFF")
#     time.sleep(1)
