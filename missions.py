import time
from figureEight import figureEight
from main import get_location_offset_meters
from dronekit import VehicleMode

def firstMission(vehicle):
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
