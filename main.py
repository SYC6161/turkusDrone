from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import random

# Bağlantıyı başlat (SITL için)
print("Drone'a bağlanılıyor...")
vehicle = connect('udp:127.0.0.1:5760', wait_ready=True)

def arm_and_takeoff(target_altitude):
    print("Drone arming...")
    while not vehicle.is_armable:
        print("Drone hazır değil...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Motorlar hazırlanıyor...")
        time.sleep(1)
    print("Kalkış yapılıyor...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(f"Yükseklik: {vehicle.location.global_relative_frame.alt:.1f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Hedef irtifaya ulaşıldı.")
            break
        time.sleep(1)

#def check_for_obstacle():
    #Sensör okunacak yolda birşey var mı  diye

def avoid_obstacle():
    print("Engelden kaçınılıyor: Sağa dönülüyor.")
    #90 derece sağa dönüş yap
    vehicle.mode = VehicleMode("GUIDED")
    condition_yaw(90)
    time.sleep(3)  # Dönüş için bekle
    print("Kaçınma tamamlandı.")

def condition_yaw(heading, relative=True):
    """
    Yaw kontrolü için MAVLink komutu
    heading: derece (0-360)
    relative: göreceli mi mutlak mı
    """
    from pymavlink import mavutil
    if relative:
        is_relative = 1  # göreceli
    else:
        is_relative = 0  # mutlak
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                  # target system, component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,# komut
        0,                                     # confirmation
        heading,                              # param 1: hedef açısı
        0,                                   # param 2: hız (deg/s)
        1,                                   # param 3: yönde (1 saat yönünde)
        is_relative,                         # param 4: relatif/mutlak
        0, 0, 0)                             # param 5-7: boş
    vehicle.send_mavlink(msg)

def goto_waypoint(lat, lon, alt):
    location = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(location)

def main_mission():
    waypoints = [
        (vehicle.location.global_frame.lat + 0.0001, vehicle.location.global_frame.lon, 10),
        (vehicle.location.global_frame.lat + 0.0001, vehicle.location.global_frame.lon + 0.0001, 10),
        (vehicle.location.global_frame.lat, vehicle.location.global_frame.lon + 0.0001, 10),
    ]

    arm_and_takeoff(10)

    for idx, point in enumerate(waypoints):
        print(f"Waypoint {idx+1}: {point}")
        goto_waypoint(*point)
        time.sleep(2)  # Yolun yarısını geldik

        # Engel kontrolü (sürekli olması gerekecek)
        if check_for_obstacle():
            avoid_obstacle()

        # Waypoint'e varış kontrolü
        while True:
            dist = get_distance_meters(vehicle.location.global_frame, LocationGlobalRelative(*point))
            print(f"Waypoint'e uzaklık: {dist:.1f}m")
            if dist < 1:
                print("Waypoint'e ulaşıldı.")
                break
            time.sleep(1)

    print("Görev tamamlandı, iniş yapılıyor.")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print(f"İniş yapılıyor... İrtifa: {vehicle.location.global_relative_frame.alt:.1f}m")
        time.sleep(1)

    vehicle.close()
    print("Drone görevini tamamladı ve bağlantı kapandı.")

def get_distance_meters(aLocation1, aLocation2):
    #İki kordinat arası mesafeyi bul
    from math import radians, cos, sin, asin, sqrt
    lat1, lon1 = aLocation1.lat, aLocation1.lon
    lat2, lon2 = aLocation2.lat, aLocation2.lon
    R = 6371000
    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    a = sin(dLat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon/2)**2
    c = 2 * asin(sqrt(a))
    return R * c

if __name__ == "__main__":
    try:
        main_mission()
    except KeyboardInterrupt:
        print("Görev iptal edildi!")
        vehicle.close()
