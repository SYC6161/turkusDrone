Türküş Drone

1. Bağlantı ve Başlangıç
   vehicle = connect('udp:127.0.0.1:5760', wait_ready=True)

   Bu satır, drone ile haberleşmek için bağlantıyı başlatır.
   wait_ready=True parametresi, drone’nun hazır olmasını bekler.

2. Kalkış Fonksiyonu: arm_and_takeoff(target_altitude)

   Drone önce uçuşa hazır mı kontrol edilir (is_armable).

   Ardından drone GUIDED moduna alınır ve motorlar arıma (hazırlama) komutuyla çalıştırılır.

   simple_takeoff() fonksiyonu ile drone belirtilen yüksekliğe kalkar.

   Drone yüksekliğe ulaşana kadar her saniye yükseklik bilgisi ekrana yazdırılır.

3. Engel Algılama ve Kaçınma
   Engel Algılama: check_for_obstacle()

   Burada gerçek sensör verisi olmadığı için rastgele sayı ile engel var/yok durumu simüle edilir.

   Eğer drone önünde 3 metreden daha yakın bir engel varsa, engel algılanmış sayılır.

   Engel Kaçınma: avoid_obstacle()

   Engel algılanınca drone sağa doğru 90 derece döner.

   condition_yaw() fonksiyonu MAVLink komutlarıyla drone’nun yönünü değiştirir.

   Basit bir manevra olarak 3 saniye bekleyip kaçınma tamamlanır.

4. Waypoint’lere Gitme: goto_waypoint(lat, lon, alt)

   Belirtilen GPS koordinatlarına (enlem, boylam, yükseklik) drone’u yönlendirir.

   simple_goto() fonksiyonu ile drone o noktaya uçar.

5. Ana Görev Döngüsü: main_mission()

   Önce arm_and_takeoff(10) ile drone 10 metreye kalkar.

   Ardından örnek olarak 3 farklı waypoint belirlenir.

   Her waypoint’e gidilirken:

   Yolun ortasında engel kontrolü yapılır,

   Engel varsa kaçınma manevrası uygulanır,

   Waypoint’e varana kadar drone konumu sürekli kontrol edilir.

   Görev tamamlandığında drone LAND moduna geçer ve güvenli iniş yapar.

   İniş tamamlanana kadar irtifa takip edilir ve motorlar kapatılır.

6. Yardımcı Fonksiyon: get_distance_meters()

   İki GPS koordinatı arasındaki mesafeyi metre cinsinden hesaplar.

   Bu sayede drone ile hedef waypoint arasındaki uzaklık ölçülür.
