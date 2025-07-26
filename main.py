from init import *
from takeoff_and_land import *

vehicle = connectMyCopter()

print("Araca Bağlanıldı.")

vehicle.mode=VehicleMode("GUIDED")

armAndTakeoff(2) #2 metre yüksel

vehicle.mode = VehicleMode("LAND") #2 metreye ulaştıktan sonra yere in

time.sleep(2)

print("Görev Sonu.")

while True:
    time.sleep(2)

vehicle.close()