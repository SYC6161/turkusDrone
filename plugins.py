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
