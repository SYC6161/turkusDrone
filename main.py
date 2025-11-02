from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
from distanceFinder import get_distance_meters
from connection import connectToDevice
from plugins import avoid_obstacle
import figureEight
# Bağlantıyı başlat (SITL için)

vehicle = None

connectToDevice(vehicle)

def armingVehicle():
    vehicle.armed = True
    vehicle.mode = VehicleMode("GUIDED")
    print("Araç mod değişikliği bekleniyor....")
    time.sleep(3)

if(vehicle.armed == False):
        print("Araç mod değişikliği başarısız tekrar deneniyor....")
        armingVehicle()
else:
    pass

if(vehicle.mode.name != "GUIDED"):
        print("Araç mod değişikliği başarısız tekrar deneniyor....")
        armingVehicle()
else:
    pass    


def arm_and_takeoff(target_altitude):
    print("Kalkış yapılıyor...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(f"Yükseklik: {vehicle.location.global_relative_frame.alt:.1f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Hedef irtifaya ulaşıldı.")
            break
        time.sleep(1)

def check_for_obstacle():
    #Sensör okunacak yolda birşey var mı  diye
    None

def get_location_offset_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Dünya yarıçapı (metre)
    
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)

    return LocationGlobalRelative(newlat, newlon, original_location.alt)


def main_mission():
    
    missionID = 0

    if(missionID == 1):
        vehicle.simple_takeoff(20)
        while True:
            print(f"Yükseklik: {vehicle.location.global_relative_frame.alt:.1f}m")
            if vehicle.location.global_relative_frame.alt >= 20 * 0.95:
                print("Hedef irtifaya ulaşıldı.")
            break
        time.sleep(1)

        print("100 metre ileri (kuzeye) gidiliyor...")
        current_location = vehicle.location.global_relative_frame
        target_location = get_location_offset_meters(current_location, dNorth=100, dEast=0)
        vehicle.simple_goto(target_location)

        for i in range(5):
            figureEight(vehicle) 

    print("Görev tamamlandı, iniş yapılıyor.")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f"İniş yapılıyor... İrtifa: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)

    vehicle.close()
    print("Drone görevini tamamladı ve bağlantı kapandı.")


if __name__ == "__main__":
    try:
        main_mission()
    except KeyboardInterrupt:
        print("Görev iptal edildi!")
        vehicle.close()
