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
    pix_count = 0
    for r in range(height):
        prev_pix = img[r][0]
        for col in range(widht):
            pix = img[r][col]
            if (pix == prev_pix):
                pix_count += 1
            else:
                img_p.append('*%d,%s\n' % (pix_count, prev_pix))
                pix_count = 1
            prev_pix = pix
        if pix_count > 0:
            img_p.append('*%d,%s\n' % (pix_count, prev_pix))
            pix_count = 0

    img_p = np.array(img_p)
    # n, s = 0, -1
    # deleted = 0
    # for i, row in enumerate(img_p):
    #     if row == '*72,0x0\n':
    #         if s == -1:
    #             s = i - deleted
    #         n += 1
    #         continue
    #     if n > 0:
    #         deleted += n
    #         img_p = np.delete(img_p, np.arange(s, s+n))
    #         img_p = np.insert(img_p, s, '*B1,%d\n'%n)
    #         img_p = np.insert(img_p, s+1, '*72,0x0\n')
    #         s, n = -1, 0
    # if n > 0:
    #     deleted += n
    #     img_p = np.delete(img_p, np.arange(s, s + n))
    #     img_p = np.insert(img_p, s, '*B1,%d\n' % n)
    #     img_p = np.insert(img_p, s + 1, '*72,0x0\n')
    #     s, n = -1, 0
    return img_p, height

def open_comm(tty_dev, speed=38400):
    print("Opening %s ...\n" % tty_dev)
    ser = serial.Serial(tty_dev,
                        baudrate=speed,
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

def control_seq(seq, height):
    out_im = Image.new("RGB", (72, height), (0, 0, 0))
    row, column = 0,0
    for j, s in enumerate(seq):
        n, color = s.split(',')

        if n == '*B1':
            column=0
            continue
        n = int(n[1:])
        color = int(color, 16)
        color = (int((color & 0xFF0000) >> 16), int((color & 0x00FF00) >> 8), int(color & 0x0000FF))
        for i in range(n):
            try:
                out_im.putpixel((column, row), color)
            except:
                print('rvrve', column, row)
            column += 1
        if column == 72:
            row+=1
            column = 0
    out_im = out_im.crop((0,0,72,row))
    out_im.save('generated_files/rec_led_seq.bmp')

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
    time.sleep(2.0)
    unit.close()
    filename = 'generated_files/led_sequence.bmp'
    stick = open_comm('COM5', speed=115200)
    img, columns = proc_img(filename)
    control_seq(img, columns)

    print(query(stick, 'd'))
    print(query(stick, 'w1,%s'%filename[3:-4], sleep=2.0))

    query(stick, '*L%d\n'%(len(img)+7))
    query(stick, '*X72\n')
    query(stick, '*G2500\n')
    query(stick, '*P8000\n')
    n = len(img)
    print('length', n)

    query(stick, '*Z%d\n'%columns)
    for i in range(len(img)):
        query(stick, img[i])

    query(stick, '*K\n')
    query(stick, '*E\n')

    query(stick, 'c0x010101\n')
    #
    start_show(unit)


