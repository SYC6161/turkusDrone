from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import argparse
import math
from distanceFinder import get_distance_meters
from plugins import avoid_obstacle, armingVehicle,takeoff
from missions import firstMission
# Bağlantıyı başlat (SITL için)

missionID = print("Lütfen Görev IDsi Giriniz. 1) 8 çizme, 2) Payload")

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

if(vehicle.armed == False):
        print("Araç mod değişikliği başarısız tekrar deneniyor....")
        armingVehicle(vehicle)

if(vehicle.mode.name != "GUIDED"):
        print("Araç mod değişikliği başarısız tekrar deneniyor....")
        armingVehicle(vehicle)



def check_for_obstacle():
    #Sensör okunacak yolda birşey var mı  diye
    pass



def main_mission(missionID):

    if(missionID == 1):
        firstMission(vehicle)


if __name__ == "__main__":
    try:
        main_mission()
    except KeyboardInterrupt:
        print("Görev iptal edildi!")
        vehicle.close()
