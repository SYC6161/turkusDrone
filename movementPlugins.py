import time
from dronekit import VehicleMode

def avoid_obstacle(condition_yaw,vehicle):
    print("Engelden kaçınılıyor: Sağa dönülüyor.")
    #90 derece sağa dönüş yap
    vehicle.mode = VehicleMode("GUIDED")
    condition_yaw(90)
    time.sleep(3)  # Dönüş için bekle
    print("Kaçınma tamamlandı.")
