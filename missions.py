import time
from figureEight import figureEight
from main import get_location_offset_meters
from dronekit import VehicleMode

def firstMission(vehicle):
    
    #Araç hedef irtifaya yükselir. (20 M)
    vehicle.simple_takeoff(20)
    while True:
        print(f"Yükseklik: {vehicle.location.global_relative_frame.alt:.1f}m")
        if vehicle.location.global_relative_frame.alt >= 20 * 0.95:
            print("Hedef irtifaya ulaşıldı.")
        break

    #Araç başlangıc noktasına gider.
    time.sleep(1)
    print("100 metre ileri (kuzeye) gidiliyor...")
    current_location = vehicle.location.global_relative_frame
    target_location = get_location_offset_meters(current_location, dNorth=100, dEast=0)
    vehicle.simple_goto(target_location)

    #Sekiz çizme sistemini söylenen kadar çalıştır.(5)
    for i in range(5):
        figureEight(vehicle) 

    #Görev bitimi ardından iniş yap.
    #TODO: Drone home pozisyonuna dönsün.
    print("Görev tamamlandı, iniş yapılıyor.")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f"İniş yapılıyor... İrtifa: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)

    #Araç kendini kapatır.
    vehicle.close()
    print("Drone görevini tamamladı ve bağlantı kapandı.")
