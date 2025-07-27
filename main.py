import init
import takeoff_and_land as TAL
import time

vehicle = init.connectMyCopter()

print("Araca Bağlanıldı.")

vehicle.mode=TAL.VehicleMode("GUIDED")

TAL.armAndTakeoff(2,vehicle) #2 metre yüksel

vehicle.mode = TAL.VehicleMode("LAND") #2 metreye ulaştıktan sonra yere in

time.sleep(2)

print("Görev Sonu.")

while True:
    time.sleep(2)

vehicle.close()
