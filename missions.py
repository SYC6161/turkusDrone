import time
from figureEight import figureEight
from plugins import get_location_offset_meters
from dronekit import VehicleMode
import cv2
import numpy as np
import sys
from yigit import *

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

def secondMission(vehicle):    

    #Servoyu Ara
    try:
        import RPi.GPIO as GPIO # type: ignore
        RPI_AVAILABLE = True
    except Exception:
        print("[WARN] RPi.GPIO bulunamadı — servo kontrolü devre dışı kalacak (test için görüntü tarafını kullanabilirsiniz).")
        RPI_AVAILABLE = False

    # Mavlink'i Ara
    try:
        from pymavlink import mavutil
        MAVLINK_AVAILABLE = True
    except Exception:
        MAVLINK_AVAILABLE = False


    if RPI_AVAILABLE:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        servo_pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz
        servo_pwm.start(LOCK_DUTY)
        time.sleep(0.5)

    def is_hexagon(contour):
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.03 * peri, True) # pyright: ignore[reportUndefinedVariable]
        if len(approx) != 6:
            return False, None
        area = cv2.contourArea(contour) # type: ignore
        if area < MIN_HEX_AREA:
            return False, None

        # Kenar uzunlukları benzer mi kontrolü
        sides = []
        for i in range(6):
            pt1 = approx[i][0]
            pt2 = approx[(i + 1) % 6][0]
            sides.append(np.linalg.norm(pt1 - pt2))
        if min(sides) < 5:
            return False, None
        ratio = max(sides) / (min(sides) + 1e-6)
        if ratio > 2.5:   # çok düzensizse eleniyor
            return False, None

        # centroid
        M = cv2.moments(contour) # type: ignore
        if M['m00'] == 0:
            return False, None
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        return True, (cx, cy), approx

    # -------------------
    # GÖRÜNTÜ DÖNGÜSÜ
    # -------------------
    cap = cv2.VideoCapture(CAM_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)

    if not cap.isOpened():
        print("[ERR] Kamera açılamadı. Çıkılıyor.")
        if RPI_AVAILABLE:
            servo_pwm.stop(); GPIO.cleanup()
        sys.exit(1)

    frame_center = (FRAME_W // 2, FRAME_H // 2)
    confirm_count = 0
    last_release_time = 0
    RELEASE_COOLDOWN = 5.0  # saniye, peş peşe yanlışlıkla tekrar bırakmayı engelle

    print("[RUN] Döngü başlatıldı. Kameradan görüntü alınıyor...")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            # ön-işlem
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5,5), 0)
            edges = cv2.Canny(blur, 60, 160)

            # küçük gürültüleri kapat
            kernel = np.ones((3,3), np.uint8)
            edges = cv2.dilate(edges, kernel, iterations=1)
            edges = cv2.erode(edges, kernel, iterations=1)

            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            found_hex = False
            hex_centroid = None
            hex_poly = None

            for cnt in contours:
                ok = False
                try:
                    ok, centroid, poly = is_hexagon(cnt)
                except Exception:
                    ok = False
                if ok:
                    found_hex = True
                    hex_centroid = centroid
                    hex_poly = poly
                    # sadece ilk uygun konturu kullan
                    break

            # Görselleştirme
            disp = frame.copy()
            cv2.circle(disp, frame_center, 4, (255,0,0), -1)
            cv2.putText(disp, f"Center: {frame_center}", (10,20), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0),1)

            if found_hex:
                cx, cy = hex_centroid
                cv2.circle(disp, (cx,cy), 4, (0,255,0), -1)
                if hex_poly is not None:
                    cv2.drawContours(disp, [hex_poly], -1, (0,255,0), 2)
                dx = cx - frame_center[0]
                dy = cy - frame_center[1]
                cv2.putText(disp, f"Offset: ({dx},{dy})", (10,40), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)

                # MAVLink'den irtifa oku (opsiyonel)
                alt = get_mav_altitude()
                if alt is not None:
                    cv2.putText(disp, f"Alt: {alt:.1f}m", (10,60), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255),1)

                # merkez kontrolü
                if abs(dx) <= PIXEL_TOLERANCE and abs(dy) <= PIXEL_TOLERANCE:
                    confirm_count += 1
                    cv2.putText(disp, f"Confirm: {confirm_count}/{CONFIRM_FRAMES}", (10,80), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
                else:
                    confirm_count = 0

                # onaylanmışsa ve cooldown doluysa bırak
                if confirm_count >= CONFIRM_FRAMES:
                    now = time.time()
                    if now - last_release_time < RELEASE_COOLDOWN:
                        cv2.putText(disp, "Cooldown aktif", (10,100), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),1)
                    else:
                        # MAV güvenlik kontrolü
                        if alt is not None:
                            if not (MIN_RELEASE_ALT <= alt <= MAX_RELEASE_ALT):
                                cv2.putText(disp, f"Alt yetersiz/çok yüksek: {alt:.1f}m", (10,100), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255),1)
                                confirm_count = 0
                            else:
                                release_payload()
                                last_release_time = now
                                break
                        else:
                            # MAV yoksa doğrudan bırak (yer testleri için)
                            release_payload()
                            last_release_time = now
                            break
            else:
                confirm_count = 0

            # göster
            cv2.imshow("Hex Center Release", disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[USER] Çıkış komutu alındı.")
                break

    except KeyboardInterrupt:
        print("[USER] KeyboardInterrupt ile çıkılıyor.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if RPI_AVAILABLE:
            set_servo_lock()
            time.sleep(0.2)
            servo_pwm.stop()
            GPIO.cleanup()
        if mav is not None:
            try:
                mav.close()
            except:
                pass
        print("[EXIT] Program sonlandı.")
