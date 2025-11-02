from dronekit import connect    
import argparse

def connectToDevice(vehicle):
    parser = argparse.ArgumentParser(description="Commands")
    parser.add_argument('--connect')
    args = parser.parse_args()

    connectionString = args.connect

    print("%s adresindeki aygıta bağlanılıyor." %connectionString)
    vehicle = connect(connectionString,wait_ready = True)