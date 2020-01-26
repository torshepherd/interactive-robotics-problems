import mmap
import struct

b = struct.pack('f', 3.14)

with open("reference.tmp", "wb") as f:
    f.write(b)

with open("reference.tmp", "r+b") as f:
    # memory-map the file, size 0 means whole file
    mm = mmap.mmap(f.fileno(), 16)
    done = False
    while not done:
        num = float(input('Type a float: '))
        for i in range(4):
            mm[4 * i:4 * (i + 1)] = struct.pack('f', num)
        if num == 0:
            done = True
    # close the map
    mm.close()
