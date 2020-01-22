import mmap
import struct
import time

prev_time = time.time()

with open("reference.tmp", "r+b") as f:
    # memory-map the file, size 0 means whole file
    mm = mmap.mmap(f.fileno(), 16)
    done = False
    while not done:
        num = struct.unpack('f', mm.readline()[0:4])
        print(num[0])
        mm.seek(0)
        if num == 0:
            done = True
            
        time_diff = time.time() - prev_time
        prev_time = time.time()
        print(time_diff)
        time.sleep(0.1)
    # close the map
    mm.close()
