from datetime import datetime
import queue
from collections import deque

def create_file_name_date():
    date_today = datetime.now()
    date_today_st = str(date_today.day) +"-"+ str(date_today.month) +"-"+ str(date_today.year)
    return date_today_st

def create_file_w_date():
    file_name_date = create_file_name_date()
    file_name = file_name_date + ".txt "
    file_date= open(file_name, "w+")
    file_date.close()
    return file_name

def write_logs (file_name_txt, log):
    file_date = open(file_name_txt, "a")
    file_date.write('\n'.join(log) + '\n')
    file_date.close()
    log.clear()

if __name__ == '__main__':
    a = create_file_w_date()
    log1 = deque()
    log1.append('drone kalkti')
    log1.append('drone indi')
    log1.append('drone kalkti')
    log1.append('drone indi')
    log1.append('gorev tamamlandi')
    write_logs(a, log1)
    log1.append('drone kalkti')
    log1.append('drone ucuyooor')
    log1.append('ucuyor')
    log1.append('birinci olucaz')
    write_logs(a,log1)

    print (log1)