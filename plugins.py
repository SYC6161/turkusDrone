from dronekit import connect, VehicleMode, LocationGlobalRelative
import math
import time

def meter_offset_to_latlon(base_lat, base_lon, dx, dy):
    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(base_lat)))
    return base_lat + dlat, base_lon + dlon

def goto_location(lat, lon, alt,vehicle):
    loc = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(loc)
    time.sleep(4)


def moveForward(meters):
    #Move forward given meters
    pass

def moveBackwards(meters):
    #Move backwards given meters
    pass

def moveClockwise(center_lat, center_lon, radius, altitude, clockwise=True, points=8):
    direction = 1
    for i in range(points + 1):
        angle = math.pi * (i / points)  # 0 to π
        angle = direction * angle
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        lat, lon = meter_offset_to_latlon(center_lat, center_lon, dx, dy)
        goto_location(lat, lon, altitude)
    pass

def moveCounterClockwise(meters):
    #Move CounterClockwise given meters
    pass

def avoid_obstacle(condition_yaw,vehicle):
    print("Engelden kaçınılıyor: Sağa dönülüyor.")
    #90 derece sağa dönüş yap
    vehicle.mode = VehicleMode("GUIDED")
    condition_yaw(90)
    time.sleep(3)  # Dönüş için bekle
    print("Kaçınma tamamlandı.")

def armingVehicle(vehicle):
    vehicle.armed = True
    vehicle.mode = VehicleMode("GUIDED")
    print("Araç mod değişikliği bekleniyor....")
    time.sleep(3)

def takeoff(target_altitude,vehicle):
    print("Kalkış yapılıyor...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        print(f"Yükseklik: {vehicle.location.global_relative_frame.alt:.1f}m")
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Hedef irtifaya ulaşıldı.")
            break
        time.sleep(1)
    
def get_location_offset_meters(original_location, dNorth, dEast):
    earth_radius = 6378137.0  # Dünya yarıçapı (metre)
    
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)

    return LocationGlobalRelative(newlat, newlon, original_location.alt)
