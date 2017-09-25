import serial
import time
import operator

from PIL import Image, BmpImagePlugin
import numpy as np

# Postup:
# 1. Navrhnout trajektorii, ktera by vystacila pro dany obraz
# 2. Zjistit velikost projekce
# 3. Projet trajektorii a nasamplovat realne pozice a natoceni chapadla
# 4. Prepocitat obrazek a nahrat do tycky
# 5. PROFIT!
#

def proc_img(file):
    im = Image.open(file)  #type: BmpImagePlugin.BmpImageFile
    im.load()
    widht, height = im.size
    img = []
    for r in range(height):
        row = []
        for col in range(widht):
            a = im.getpixel((col, r))
            row.append(str("0x%X" % ((a[0] << 16) + (a[1] << 8) + a[2])))
        img.append(row)
    print 'ready'

    img_p = []
    for r in range(height):
        pix, prev_pix = 0x0, 0x0
        pix_count = 0
        for col in range(widht):
            pix = img[r][col]
            if (pix == prev_pix or col == 0) and col < widht - 1:
                pix_count += 1
            else:
                pix_count += 1
                img_p.append('*%d,%s\n' % (pix_count, prev_pix))
                pix_count = 0
            prev_pix = pix
    return img_p

def open_comm(tty_dev):
    print("Opening %s ...\n" % tty_dev)
    ser = serial.Serial(tty_dev,
                        baudrate=19200,
                        bytesize=serial.EIGHTBITS,
                        parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE,
                        rtscts=True,
                        timeout=0.01)
    return ser

def query(rcon, query, sleep=0.0):
    buf = '\n'
    rcon.write('\n' + query + '\n')
    if sleep > 0:
        time.sleep(sleep)
    buf = rcon.read(1024)
    return buf

def power_led(rcon):
    rcon.write('RELEASE:\n')
    rcon.write('PURGE:\n')
    rcon.write('REGMEH:32730\n')
    rcon.write('REGIH:0\n')
    rcon.write('REGDH:0\n')
    rcon.write('REGPH:100\n')
    rcon.write('GH:0\n')
    time.sleep(3.0)
    rcon.write('GH:327\n')

def start_show(rcon):
    rcon.write('GH:30\n')
    time.sleep(1.0)
    rcon.write('GH:327\n')
    
if __name__ == '__main__':
    unit = open_comm('COM3')
    power_led(unit)
    time.sleep(5.0)
    filename = 'img\\pikachu.bmp'
    stick = open_comm('COM5')
    img = proc_img(filename)
    print query(stick, 'd')
    print query(stick, 'w1,%s'%filename[:-4], sleep=2.0)
    query(stick, '*L10\n')
    query(stick, '*X72\n')
    query(stick, '*G2500\n')
    query(stick, '*P2000\n')
    query(stick, '*Z10000\n')
    # query(stick, '*C0x010101\n')
    for i in range(len(img)):
        query(stick, img[i])
    query(stick, '*K\n')
    query(stick, '*E\n')

    query(stick, 'c0x010101\n')

    start_show(unit)


