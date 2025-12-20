from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import math
import pymavlink
from distanceFinder import get_distance_meters
from plugins import avoid_obstacle, armingVehicle,takeoff
from missions import firstMission, secondMission
# Bağlantıyı başlat (SITL için)

missionID = input("Lütfen Görev IDsi Giriniz. 1) 8 çizme, 2) Payload")

#vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=57600)

#Drone'a bağlantı kur
parser = argparse.ArgumentParser(description="Commands")
parser.add_argument('--connect')
args = parser.parse_args()
connectionString = args.connect
print("%s adresindeki aygıta bağlanılıyor." %connectionString)
vehicle = connect(connectionString,wait_ready = True)


#Drone'u uçuşa hazır hale getir
armingVehicle(vehicle)
print("Araç hazırlanıyor.....")
time.sleep(1)
while vehicle.armed == False and vehicle.mode.name != "GUIDED":

    armingVehicle(vehicle)
    if (vehicle.armed == True and vehicle.mode.name != "GUIDED"):
        break


def main_mission(missionID):
    if(missionID == 1):
        firstMission(vehicle)
    elif(missionID == 2):
        secondMission(vehicle)
    else:
        print("Yanlış Görev Seçildi...")

main_mission(missionID)

if __name__ == "__main__":  
    if KeyboardInterrupt:
        print("Görev iptal edildi!")
        vehicle.close()
