print("initializing Drone.........")

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exceptions
import argparse

#GroundControl ile bağlantı kur
def connectMyCopter():
    parser = argparse.ArgumentParser(description="commands")
    parser.add_argument('--connect')
    args = parser.parse_args()

    connectionString = args.connect

    vehicle = connect(connectionString, wait_ready=True)

    return vehicle