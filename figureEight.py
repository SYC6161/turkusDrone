from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math

def goto_location(lat, lon, alt,vehicle):
    loc = LocationGlobalRelative(lat, lon, alt)
    vehicle.simple_goto(loc)
    time.sleep(4)

# --- Dereceye göre offset hesapla (lat/lon) ---
def meter_offset_to_latlon(base_lat, base_lon, dx, dy):
    dlat = dy / 111111
    dlon = dx / (111111 * math.cos(math.radians(base_lat)))
    return base_lat + dlat, base_lon + dlon

# --- Belirli merkez etrafında yarım daire uçuşu ---
def fly_half_circle(center_lat, center_lon, radius, altitude, clockwise=True, points=8):
    direction = -1 if clockwise else 1
    for i in range(points + 1):
        angle = math.pi * (i / points)  # 0 to π
        angle = direction * angle
        dx = radius * math.cos(angle)
        dy = radius * math.sin(angle)
        lat, lon = meter_offset_to_latlon(center_lat, center_lon, dx, dy)
        goto_location(lat, lon, altitude)

def figureEight(vehicle): 

    half_dist = 100  
    radius = 20      

    home = vehicle.location.global_relative_frame
    center_lat = home.lat
    center_lon = home.lon    

    # Direk A (solda)
    X_lat, X_lon = meter_offset_to_latlon(center_lat, center_lon, -half_dist, 0)

    # Direk B (sağda)
    Y_lat, Y_lon = meter_offset_to_latlon(center_lat, center_lon, half_dist, 0)

    print("X direği etrafında dönülüyor...")
    fly_half_circle(X_lat, X_lon, radius, 15, clockwise=True)

    # B direğine geç
    print("Y direğine geçiliyor...")
    goto_location(Y_lat, Y_lon, 15,vehicle)

    # B direği etrafında saat yönünün tersine yarım daire
    print("Y direği etrafında dönülüyor...")
    fly_half_circle(Y_lat, Y_lon, radius, 15, clockwise=False)
