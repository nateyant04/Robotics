#Sample movement to ensure communication with the 3D printer
import serial
import time

# maybe adjust these
pen_up_z = 10
pen_down_z = 0

def home_printer(ser):
    ser.write(b'G28\n')
    time.sleep(2)  # Wait for the printer to process the command

def coord_mode_absolute(ser, mode=True):
    if mode:
        ser.write(b'G90\n')  # Set to absolute positioning
    else:
        ser.write(b'G91\n')  # Set to relative positioning
    time.sleep(1)  # Wait for the printer to process the command

def move_to(ser, x=None, y=None, z=None, feedrate=1500):
    command = 'G1'
    if x is not None:
        command += f' X{x}'
    if y is not None:
        command += f' Y{y}'
    if z is not None:
        command += f' Z{z}'
    command += f' F{feedrate}\n'
    ser.write(command.encode())
    time.sleep(2)  # Wait for the printer to process the command

def pen_up(ser):
    move_to(ser, z=pen_up_z)  # Move the pen up (adjust Z value as needed)
    time.sleep(1)  # Wait for the printer to process the command

def pen_down(ser):
    move_to(ser, z=pen_down_z)  # Move the pen down (adjust Z value as needed)
    time.sleep(1)  # Wait for the printer to process the command

def draw_polygon(ser, points):
    pen_up(ser)
    time.sleep(1)
    move_to(ser, x=points[0][0], y=points[0][1])
    pen_down(ser)
    time.sleep(1)
    for point in points[1:]:
        move_to(ser, x=point[0], y=point[1])
        time.sleep(0.25)
    pen_up(ser)

def main():
    port = 'COM3' #Set this to your actual COM port!
    baudrate = 115200  # Common baudrate for 3D printers, adjust if needed

    try:
        # Open the serial connection
        ser = serial.Serial(port, baudrate, timeout=2)
        print(f"Connected to 3D printer on {port}")

        # Allow the printer to initialize
        time.sleep(2)

        coord_mode_absolute(ser)
        home_printer(ser)
        coord_mode_absolute(ser)

        pen_up(ser)
        move_to(ser, x=50, y=50)

        # draw square
        draw_polygon(ser, [(50, 50), (100, 50), (100, 100), (50, 100), (50, 50)])

        
        #move_to(ser, x=100, y=150)

        # draw a star
        #draw_polygon(ser, [(100, 150), (110, 170), (130, 170), (115, 180), (120, 200), (100, 190), (80, 200), (85, 180), (70, 170), (90, 170), (100, 150)])


        # flush the serial buffer
        ser.flush()
        print("Commands sent successfully.")

    except serial.SerialException as e:
        print(f"Error: {e}")

    finally:
        # Close the connection
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Connection closed.")

if __name__ == "__main__":
    main()
