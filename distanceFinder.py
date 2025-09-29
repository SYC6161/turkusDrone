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
