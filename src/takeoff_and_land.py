#Dependincies
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import socket
import exceptions
import argparse

def armAndTakeoff(targetAltitude,vehicle):
    while not vehicle.isArmable:
        print("Araç Bekleniyor.")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    while Vehicle.mode != "GUIDED":
        print("Araçın Guided Moduna Girmesi Bekleniyor.")
        time.sleep(1)
    
    vehicle.simple_takeoff(targetAltitude)

    while True:
        print("Mevcut Yükseklik: %d"%vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= targetAltitude *0.95:
            break
        time.sleep(1)
    print("Hedef Yüksekliğe Ulaşıldı.")
    return None