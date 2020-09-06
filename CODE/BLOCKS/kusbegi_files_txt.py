def write_coordinates (lat, lon):
    file = open('pool_coordinates.txt', 'w', encoding="utf-8")
    file.write(str(lat) + "\n")
    file.write(str(lon) )
    file.close()

def read_coordinates():
    file = open('pool_coordinates.txt','r', encoding="utf-8")
    lan1 = float(file.readline())
    lon1 = float(file.readline())
    file.close()
    return lan1, lon1

if __name__ == '__main__':
    lat = 3.121324
    lon = -3.32488
    write_coordinates(lat, lon)
    print(read_coordinates())

