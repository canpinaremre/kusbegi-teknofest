def write_coordinates (file_path,lat, lon):
    file = open('file_path', 'w', encoding="utf-8")
    file.write(str(lat) + "\n")
    file.write(str(lon) )
    file.close()

def read_coordinates(file_path):
    file = open('file_path','r', encoding="utf-8")
    lan1 = float(file.readline())
    lon1 = float(file.readline())
    file.close()
    return lan1, lon1

if __name__ == '__main__':
    file_path = "C:\\Users\\ASUS\\PycharmProjects\\pythonProject1\\venv\\pool_coordinates.txt"
    lat = 3.121324
    lon = -3.32488
    write_coordinates(file_path, lat, lon)
    print(read_coordinates(file_path))

