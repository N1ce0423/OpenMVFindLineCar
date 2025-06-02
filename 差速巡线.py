import sensor
import time
from machine import UART, LED
from pid import PID

sensor.reset()  # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565)  # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QVGA)  # Set frame size to QVGA (320x240)
sensor.skip_frames(time=2000)  # Wait for settings take effect.
sensor.set_vflip(True)
sensor.set_hmirror(True)
led = LED("LED_BLUE")
led.on()
clock = time.clock()  # Create a clock object to track the FPS.

black = (32, 100, -128, 127, -128, 127)
uart = UART(3, 115200, bits=8, parity=None, stop=1, timeout_char=1000)
x_pid = PID(p=1.5, i=0, imax=90)
last_x = 160
isMinus = 0

def find_line(blobs):
    global last_x
    for blob in blobs:
        if blob[2] < 60 and abs(blob.cx() - last_x) < 20:
            line_blob = blob
            last_x = blob.cx()
            return line_blob
    return 1;

while True:
    img = sensor.snapshot()  # Take a picture and return the image.
    blobs = img.find_blobs([black], invert=True, roi = [0,100,320,48])
    img.draw_rectangle([0,100,320,48])
    if blobs:
        line_blob = find_line(blobs)
        if line_blob == 1:
            print("No line is found")
            continue
        img.draw_rectangle(line_blob[0:4], color = (0, 255, 0))
        x_error = line_blob.cx() - img.width() / 2
        x_output = x_pid.get_pid(x_error, 1)
        if x_output < 0:
            isMinus = 1
            x_output = -1 * x_output
        else:
            isMinus = 0
        if x_output > 252:
            x_output = 252
        data = bytearray([0xff, 0xfe, isMinus, int(x_output), 0xfd])
        uart.write(data)
    else:
        data = bytearray([0xff, 0xfe, 0, 0, 0xfd])
        uart.write(data)
