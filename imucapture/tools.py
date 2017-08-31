
#G = 9.80665
G = 9.81
PI = 3.14159265358979323846264338327

def meters_per_second_squared_to_gs(mps2):
    return (mps2 / G)

def gs_to_meters_per_second_squared(gs):
    return (gs * G)

def radians_to_degrees(radians):
    return (radians * (180.0 / PI))

def degrees_to_radians(degrees):
    return (degrees * (PI / 180.0))
