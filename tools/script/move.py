import argparse
import os
import time

import numpy as np

from tools.utils import SerialDevice, setup_logging

#  python -m tools.script.move --file trajectory.npy --port COM5 --baudrate 115200

path = os.path.abspath("tools/logger_config.json")
setup_logging(path)


def send_angles(device: SerialDevice, angles) -> None:
    angles_str = ",".join([f"{angle:.2f}" for angle in angles])
    command = angles_str + "\n"
    device.write_data(command)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="")
    parser.add_argument(
        "--file", required=True, help="Path to the .npy file containing the trajectory"
    )
    parser.add_argument(
        "--port", required=True, help="The port to which the device is connected"
    )
    parser.add_argument(
        "--baudrate", type=int, default=9600, help="Data transfer rate (default is 9600)"
    )
    parser.add_argument(
        "--timeout", type=int, default=1, help="Connection timeout (default is 1 sec)"
    )

    args = parser.parse_args()

    if not os.path.isfile(args.file):
        print(f"Error: File '{args.file}' does not exist.")
        exit(1)
    
    if not args.file.endswith('.npy'):
        print(f"Error: File '{args.file}' is not a .npy file.")
        exit(1)

    device = SerialDevice(
        port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout
    )
    device.connect()

    if not device.is_connected():
        print("Failed to connect to the device")
        exit()
    
    try:
        start_time = time.time()
        # Load trajectory data from the .npy file
        angles_history_rad = np.load(args.file)

        # Convert angles from radians to degrees and adjust by +90 degrees
        angles_history = np.round(np.degrees(angles_history_rad) + 90, 2)

        # Main loop to send trajectory data to the device
        for angles in angles_history:
            while True:
                # Check if the device is ready to receive new commands
                device.write_data("STATUS\n")
                status = device.read_data()
                
                # If the device is ready, send the angles
                if "READY" in status:
                    send_angles(device, angles)
                    break
                
                # If the device is not ready, wait before checking again
                time.sleep(0.01)
            
            # Delay between sending commands for smooth motion
            time.sleep(0.01)
        
        time.sleep(2)

        end_time = time.time()

        device.disconnect()

        print(f"Time: {(end_time - start_time):.2f} sec.")
    except KeyboardInterrupt:
        print("\nExiting...")
        device.disconnect()
