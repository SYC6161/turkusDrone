import cv2
import numpy as np
import time
import argparse
import sys

# -------------------
# KONFIGÜRASYON
# -------------------
SERVO_PIN = 17              # BCM pin (GPIO17)
LOCK_DUTY = 2.5             # kilit pozisyonu (duty cycle %)
RELEASE_DUTY = 7.5          # bırakma pozisyonu (duty cycle %)
RELEASE_TIME = 1.0          # servo açık kalma süresi (saniye)

CAM_INDEX = 0               # kamera indeksi (0 = /dev/video0)
FRAME_W = 640
FRAME_H = 480

PIXEL_TOLERANCE = 30        # merkezden kabul edilebilir piksel sapma (x ve y)
CONFIRM_FRAMES = 6         # art arda kaç karede merkezde görünmeli
MIN_HEX_AREA = 800         # kontur alanı eşiği (gürültü filtreleme)

# MAVLink güvenlik (opsiyonel) -- bağlantı /dev/ttyAMA0 örnek olsun.
USE_MAVLINK = True
MAV_DEVICE = '/dev/ttyAMA0'
MAV_BAUD = 57600
MIN_RELEASE_ALT = 0.5       # metre (güvenlik) - altıgen algılanınca minimum yükseklik
MAX_RELEASE_ALT = 50.0      # maksimum izin verilen yükseklik

def set_servo_lock(RPI_AVAILABLE,servo_pwm):
    if RPI_AVAILABLE:
        servo_pwm.ChangeDutyCycle(LOCK_DUTY)

def set_servo_release(RPI_AVAILABLE,servo_pwm):
    if RPI_AVAILABLE:
        servo_pwm.ChangeDutyCycle(RELEASE_DUTY)

def release_payload():
    print("[ACTION] Yük bırakılıyor: servo hareket ediyor.")
    set_servo_release()
    time.sleep(RELEASE_TIME)
    set_servo_lock()
    print("[ACTION] Bırakma tamamlandı. Servo kilit pozisyonunda.")

def get_mav_altitude(mav):
    """MAVLink varsa son GLOBAL_POSITION_INT veya VFR_HUD'dan irtifa almaya çalış"""
    if not USE_MAVLINK or mav is None:
        return None
    try:
        # non-blocking read of GLOBAL_POSITION_INT
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg:
            # msg.relative_alt is in mm, alt in meters:
            if hasattr(msg, 'relative_alt'):
                return float(msg.relative_alt) / 1000.0
            elif hasattr(msg, 'alt'):
                return float(msg.alt) / 1000.0
    except Exception:
        pass
    return None