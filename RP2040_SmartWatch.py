#Digital Smart Watch
#Shout out to Tony Goodhew for the base code and gfx effects
#https://www.instructables.com/Digital-Watch-Display-MicroPython/
#Davis Dixon 10/20/24
#Feel free to use
from machine import Pin,I2C,SPI,PWM,ADC
import math
import time
import utime
import framebuf

#=================Back end init=================
DC = 8 
CS = 9 
SCK = 10 
MOSI = 11 
RST = 12 
BL = 25 
Vbat_Pin = 29
width = 240
height = 240
I2C_SDL = 7
I2C_SDA = 6


class LCD_1inch28(framebuf.FrameBuffer): # Waveshare RP2040 1.28" IPS LCD Board Driver - Round Display
    def __init__(self):
        self.width = 240
        self.height = 240
        
        self.cs = Pin(CS,Pin.OUT)
        self.rst = Pin(RST,Pin.OUT)
        
        self.cs(1)
        self.spi = SPI(1,100_000_000,polarity=0, phase=0,sck=Pin(SCK),mosi=Pin(MOSI),miso=None)
        self.dc = Pin(DC,Pin.OUT)
        self.dc(1)
        self.buffer = bytearray(self.height * self.width * 2)
        super().__init__(self.buffer, self.width, self.height, framebuf.RGB565)
        self.init_display()
        
        self.red   =   0x07E0
        self.green =   0x001f
        self.white =   0xffff
        
        self.fill(self.white)
        self.show()

        self.pwm = PWM(Pin(BL))
        self.pwm.freq(5000)
        
    def write_cmd(self, cmd):
        self.cs(1)
        self.dc(0)
        self.cs(0)
        self.spi.write(bytearray([cmd]))
        self.cs(1)

    def write_data(self, buf):
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(bytearray([buf]))
        self.cs(1)
    def set_bl_pwm(self,duty):
        self.pwm.duty_u16(duty)#max 65535
    def init_display(self):
        """Initialize display"""  
        self.rst(1)
        time.sleep(0.01)
        self.rst(0)
        time.sleep(0.01)
        self.rst(1)
        time.sleep(0.05)
        
        self.write_cmd(0xEF)
        self.write_cmd(0xEB)
        self.write_data(0x14) 
        
        self.write_cmd(0xFE) 
        self.write_cmd(0xEF) 

        self.write_cmd(0xEB)
        self.write_data(0x14) 

        self.write_cmd(0x84)
        self.write_data(0x40) 

        self.write_cmd(0x85)
        self.write_data(0xFF) 

        self.write_cmd(0x86)
        self.write_data(0xFF) 

        self.write_cmd(0x87)
        self.write_data(0xFF)

        self.write_cmd(0x88)
        self.write_data(0x0A)

        self.write_cmd(0x89)
        self.write_data(0x21) 

        self.write_cmd(0x8A)
        self.write_data(0x00) 

        self.write_cmd(0x8B)
        self.write_data(0x80) 

        self.write_cmd(0x8C)
        self.write_data(0x01) 

        self.write_cmd(0x8D)
        self.write_data(0x01) 

        self.write_cmd(0x8E)
        self.write_data(0xFF) 

        self.write_cmd(0x8F)
        self.write_data(0xFF) 


        self.write_cmd(0xB6)
        self.write_data(0x00)
        self.write_data(0x20)

        self.write_cmd(0x36)
        self.write_data(0x98)

        self.write_cmd(0x3A)
        self.write_data(0x05) 


        self.write_cmd(0x90)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x08) 

        self.write_cmd(0xBD)
        self.write_data(0x06)
        
        self.write_cmd(0xBC)
        self.write_data(0x00)

        self.write_cmd(0xFF)
        self.write_data(0x60)
        self.write_data(0x01)
        self.write_data(0x04)

        self.write_cmd(0xC3)
        self.write_data(0x13)
        self.write_cmd(0xC4)
        self.write_data(0x13)

        self.write_cmd(0xC9)
        self.write_data(0x22)

        self.write_cmd(0xBE)
        self.write_data(0x11) 

        self.write_cmd(0xE1)
        self.write_data(0x10)
        self.write_data(0x0E)

        self.write_cmd(0xDF)
        self.write_data(0x21)
        self.write_data(0x0c)
        self.write_data(0x02)

        self.write_cmd(0xF0)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF1)    
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37)  
        self.write_data(0x6F)


        self.write_cmd(0xF2)   
        self.write_data(0x45)
        self.write_data(0x09)
        self.write_data(0x08)
        self.write_data(0x08)
        self.write_data(0x26)
        self.write_data(0x2A)

        self.write_cmd(0xF3)   
        self.write_data(0x43)
        self.write_data(0x70)
        self.write_data(0x72)
        self.write_data(0x36)
        self.write_data(0x37) 
        self.write_data(0x6F)

        self.write_cmd(0xED)
        self.write_data(0x1B) 
        self.write_data(0x0B) 

        self.write_cmd(0xAE)
        self.write_data(0x77)
        
        self.write_cmd(0xCD)
        self.write_data(0x63)


        self.write_cmd(0x70)
        self.write_data(0x07)
        self.write_data(0x07)
        self.write_data(0x04)
        self.write_data(0x0E) 
        self.write_data(0x0F) 
        self.write_data(0x09)
        self.write_data(0x07)
        self.write_data(0x08)
        self.write_data(0x03)

        self.write_cmd(0xE8)
        self.write_data(0x34)

        self.write_cmd(0x62)
        self.write_data(0x18)
        self.write_data(0x0D)
        self.write_data(0x71)
        self.write_data(0xED)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x0F)
        self.write_data(0x71)
        self.write_data(0xEF)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x63)
        self.write_data(0x18)
        self.write_data(0x11)
        self.write_data(0x71)
        self.write_data(0xF1)
        self.write_data(0x70) 
        self.write_data(0x70)
        self.write_data(0x18)
        self.write_data(0x13)
        self.write_data(0x71)
        self.write_data(0xF3)
        self.write_data(0x70) 
        self.write_data(0x70)

        self.write_cmd(0x64)
        self.write_data(0x28)
        self.write_data(0x29)
        self.write_data(0xF1)
        self.write_data(0x01)
        self.write_data(0xF1)
        self.write_data(0x00)
        self.write_data(0x07)

        self.write_cmd(0x66)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0xCD)
        self.write_data(0x67)
        self.write_data(0x45)
        self.write_data(0x45)
        self.write_data(0x10)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)

        self.write_cmd(0x67)
        self.write_data(0x00)
        self.write_data(0x3C)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x01)
        self.write_data(0x54)
        self.write_data(0x10)
        self.write_data(0x32)
        self.write_data(0x98)

        self.write_cmd(0x74)
        self.write_data(0x10)
        self.write_data(0x85)
        self.write_data(0x80)
        self.write_data(0x00) 
        self.write_data(0x00) 
        self.write_data(0x4E)
        self.write_data(0x00)
        
        self.write_cmd(0x98)
        self.write_data(0x3e)
        self.write_data(0x07)

        self.write_cmd(0x35)
        self.write_cmd(0x21)

        self.write_cmd(0x11)
        time.sleep(0.12)
        self.write_cmd(0x29)
        time.sleep(0.02)
        
        self.write_cmd(0x21)

        self.write_cmd(0x11)

        self.write_cmd(0x29)

    def show(self):
        self.write_cmd(0x2A)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xef)
        
        self.write_cmd(0x2B)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0x00)
        self.write_data(0xEF)
        
        self.write_cmd(0x2C)
        
        self.cs(1)
        self.dc(1)
        self.cs(0)
        self.spi.write(self.buffer)
        self.cs(1)
        
    def turn_off(self):
        self.rst(0)
        
        
class QMI8658(object):
    def __init__(self,address=0X6B):
        self._address = address
        self._bus = I2C(id=1,scl=Pin(I2C_SDL),sda=Pin(I2C_SDA),freq=100_000)
        bRet=self.WhoAmI()
        if bRet :
            self.Read_Revision()
        else    :
            return NULL
        self.Config_apply()

    def _read_byte(self,cmd):
        rec=self._bus.readfrom_mem(int(self._address),int(cmd),1)
        return rec[0]
    def _read_block(self, reg, length=1):
        rec=self._bus.readfrom_mem(int(self._address),int(reg),length)
        return rec
    def _read_u16(self,cmd):
        LSB = self._bus.readfrom_mem(int(self._address),int(cmd),1)
        MSB = self._bus.readfrom_mem(int(self._address),int(cmd)+1,1)
        return (MSB[0] << 8) + LSB[0]
    def _write_byte(self,cmd,val):
        self._bus.writeto_mem(int(self._address),int(cmd),bytes([int(val)]))
        
    def WhoAmI(self):
        bRet=False
        if (0x05) == self._read_byte(0x00):
            bRet = True
        return bRet
    def Read_Revision(self):
        return self._read_byte(0x01)
    def Config_apply(self):
        # REG CTRL1
        self._write_byte(0x02,0x60)
        # REG CTRL2 : QMI8658AccRange_8g  and QMI8658AccOdr_1000Hz
        self._write_byte(0x03,0x23)
        # REG CTRL3 : QMI8658GyrRange_512dps and QMI8658GyrOdr_1000Hz
        self._write_byte(0x04,0x53)
        # REG CTRL4 : No
        self._write_byte(0x05,0x00)
        # REG CTRL5 : Enable Gyroscope And Accelerometer Low-Pass Filter 
        self._write_byte(0x06,0x11)
        # REG CTRL6 : Disables Motion on Demand.
        self._write_byte(0x07,0x00)
        # REG CTRL7 : Enable Gyroscope And Accelerometer
        self._write_byte(0x08,0x03)

    def Read_Raw_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_timestamp = self._read_block(0x30,3)
        raw_acc_xyz=self._read_block(0x35,6)
        raw_gyro_xyz=self._read_block(0x3b,6)
        raw_xyz=self._read_block(0x35,12)
        timestamp = (raw_timestamp[2]<<16)|(raw_timestamp[1]<<8)|(raw_timestamp[0])
        for i in range(6):
            # xyz[i]=(raw_acc_xyz[(i*2)+1]<<8)|(raw_acc_xyz[i*2])
            # xyz[i+3]=(raw_gyro_xyz[((i+3)*2)+1]<<8)|(raw_gyro_xyz[(i+3)*2])
            xyz[i] = (raw_xyz[(i*2)+1]<<8)|(raw_xyz[i*2])
            if xyz[i] >= 32767:
                xyz[i] = xyz[i]-65535
        return xyz
    def Read_XYZ(self):
        xyz=[0,0,0,0,0,0]
        raw_xyz=self.Read_Raw_XYZ()  
        #QMI8658AccRange_8g
        acc_lsb_div=(1<<12)
        #QMI8658GyrRange_512dps
        gyro_lsb_div = 64
        for i in range(3):
            xyz[i]=raw_xyz[i]/acc_lsb_div#(acc_lsb_div/1000.0)
            xyz[i+3]=raw_xyz[i+3]*1.0/gyro_lsb_div
        return xyz

def colour(R,G,B): # Convert RGB888 to RGB565
       return (((G&0b00011100)<<3) +((B&0b11111000)>>3)<<8) + (R&0b11111000)+((G&0b11100000)>>5)

LCD = LCD_1inch28()            #=============== Initialise the display ===================
LCD.set_bl_pwm(5535)          # Brightness
qmi8658=QMI8658()             # Initialise gyro accl
Vbat= ADC(Pin(Vbat_Pin))      # Lipo voltage pin
#=================End of innit=================

# ========== Start of Triangles code =============
# Modified from https://github.com/SpiderMaf/PiPicoDsply/blob/main/filled-triangles.py
# To work on WaveShare Pi Pico displays
# ========== Version 2 FIXED ! 23 May 2022 ==========
class Point:
    def __init__(self,x,y):
        self.X=x
        self.Y=y
    def __str__(self):
        return "Point(%s,%s)"%(self.X,self.Y)
        
class Triangle:
    def __init__(self,p1,p2,p3):
        self.P1=p1
        self.P2=p2
        self.P3=p3

    def __str__(self):
        return "Triangle(%s,%s,%s)"%(self.P1,self.P2,self.P3)
    
    def draw(self):
        print("I should draw now")
        self.fillTri()
    # Filled triangle routines ported from http://www.sunshine2k.de/coding/java/TriangleRasterization/TriangleRasterization.html      
    def sortVerticesAscendingByY(self):    
        if self.P1.Y > self.P2.Y:
            vTmp = self.P1
            self.P1 = self.P2
            self.P2 = vTmp
        
        if self.P1.Y > self.P3.Y:
            vTmp = self.P1
            self.P1 = self.P3
            self.P3 = vTmp

        if self.P2.Y > self.P3.Y:
            vTmp = self.P2
            self.P2 = self.P3
            self.P3 = vTmp
        
    def fillTri(self,c):
        self.sortVerticesAscendingByY()
        if self.P2.Y == self.P3.Y:
            fillBottomFlatTriangle(self.P1, self.P2, self.P3,c)
        else:
            if self.P1.Y == self.P2.Y:
                fillTopFlatTriangle(self.P1, self.P2, self.P3,c)
            else:
                newx = int(self.P1.X + (float(self.P2.Y - self.P1.Y) / float(self.P3.Y - self.P1.Y)) * (self.P3.X - self.P1.X))
                newy = self.P2.Y                
                pTmp = Point( newx,newy )
#                print(pTmp)
                fillBottomFlatTriangle(self.P1, self.P2, pTmp,c)
                fillTopFlatTriangle(self.P2, pTmp, self.P3,c)

def fillBottomFlatTriangle(p1,p2,p3,c):
    
#    print("BF",p1,p2,p3)
    if p2.Y > p3.Y:
        ty = p3.Y
        p3.Y = p2.Y
        p2.Y = ty
        tx = p3.X
        p3.X = p2.X
        p2.X = tx
        print(p1,p2,p3)
    
    slope1 = float(p2.X - p1.X) / float (p2.Y - p1.Y)
    slope2 = float(p3.X - p1.X) / float (p3.Y - p1.Y)

    x1 = p1.X
    x2 = p1.X + 0.5
#    print("B",p1.Y,p2.Y)
    for scanlineY in range(p1.Y,p2.Y):
#        print(scanlineY)
#        LCD.pixel_span(int(x1), scanlineY, int(x2)-int(x1))   # Switch pixel_span() to hline() / Pimoroni to WS
        LCD.hline(int(x1),scanlineY, int(x2)-int(x1),c)        
        LCD.hline(int(x2),scanlineY, -(int(x2)-int(x1)),c)
#        LCD.show()          #                  Here and below        
#        utime.sleep(0.1)    #     <===== Uncomment to see how graphic elements are drawn
        x1 += slope1
        x2 += slope2
#    LCD.show()              #                  LCD.show() and utime.sleep(0.1)
def fillTopFlatTriangle(p1,p2,p3,c):
#    print("TF",p1,p2,p3)
    slope1 = float(p3.X - p1.X) / float(p3.Y - p1.Y)
    slope2 = float(p3.X - p2.X) / float(p3.Y - p2.Y)

    x1 = p3.X
    x2 = p3.X + 0.5
#    print("T",p3.Y,p1.Y-1)
    for scanlineY in range (p3.Y,p1.Y-1,-1):
#        print(scanlineY)
#        LCD.pixel_span(int(x1), scanlineY, int(x2)-int(x1))  # Switch pixel_span() to hline() / Pimoroni to WS
        LCD.hline(int(x1),scanlineY, int(x2)-int(x1)+1,c)        
        LCD.hline(int(x2),scanlineY, -(int(x2)-int(x1)-1),c)
#        LCD.show()
#        utime.sleep(0.1)
        x1 -= slope1
        x2 -= slope2
#    LCD.show()            
# ============== End of Triangles Code ===============

# =========== New GFX Routines ============
def clear(c):
    LCD.fill(c)
    
def triangle(x1,y1,x2,y2,x3,y3,c): # Draw outline triangle
    LCD.line(x1,y1,x2,y2,c)
    LCD.line(x2,y2,x3,y3,c)
    LCD.line(x3,y3,x1,y1,c)
    
def tri_filled(x1,y1,x2,y2,x3,y3,c): # Draw filled triangle
 
    t=Triangle(Point(x1,y1),Point(x2,y2),Point(x3,y3)) # Define corners
    t.fillTri(c) # Call main code block  

def circle(x,y,r,c):
    LCD.hline(x-r,y,r*2,c)
    for i in range(1,r):
        a = int(math.sqrt(r*r-i*i)) # Pythagoras!
        LCD.hline(x-a,y+i,a*2,c) # Lower half
        LCD.hline(x-a,y-i,a*2,c) # Upper half

def ring(x,y,r,c):
    LCD.pixel(x-r,y,c)
    LCD.pixel(x+r,y,c)
    LCD.pixel(x,y-r,c)
    LCD.pixel(x,y+r,c)
    for i in range(1,r):
        a = int(math.sqrt(r*r-i*i))
        LCD.pixel(x-a,y-i,c)
        LCD.pixel(x+a,y-i,c)
        LCD.pixel(x-a,y+i,c)
        LCD.pixel(x+a,y+i,c)
        LCD.pixel(x-i,y-a,c)
        LCD.pixel(x+i,y-a,c)
        LCD.pixel(x-i,y+a,c)
        LCD.pixel(x+i,y+a,c)

#===== start of numbers and letter=======
def numberZero(x,y,c,bg):
    y += 5
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of the six
    for i in range(65):
        if(i % 5 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+16,yy+(i%5)-8,bg)


def numberOne(x,y,c,z):
    xx = x
    yy = y
    #base of the one
    for i in range(76):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #nose of the one
    for i in range(15):
        if(i % 5 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+16,yy+(i%5)-8,c)
    #finishing touch
    LCD.pixel(x+16,y-5,c)
    LCD.pixel(x+17,y-5,c)
    LCD.pixel(x+16,y-6,c)

def numberTwo(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of two
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+15,yy+(i%7)-11,bg)
    #bg in bottom of two
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+12,yy+(i%7)-7,bg)
        
    
def numberThree(x,y,c,z):
    xx = x
    yy = y
    #backbone of the three
    for i in range(76):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c) 
    #bottom nub
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+4,yy+(i%7)-11,c) 
    #middle nub
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+16,yy+(i%7)-9,c)
    #top nub
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+27,yy+(i%7)-11,c) 

def numberFour(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at bottom of four
    for i in range(42):
        if(i % 6 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+7,yy+(i%6)-11,bg)
    #bg at top of four
    for i in range(28):
        if(i % 4 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+26,yy+(i%4)-8,bg)
    
def numberFive(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of five
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+15,yy+(i%7)-7,bg)
    #bg in bottom of five
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+11,yy+(i%7)-11,bg)



def numberSix(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of six
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+15,yy+(i%7)-7,bg)
    #bg in middle of six
    for i in range(16):
        if(i % 4 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+11,yy+(i%4)-7,bg)

def numberSeven(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    
    #left side of seven
    yy = y
    for i in range(40):
        if(i % 5 == 0):
            xx -= 1
            if(i % 10 == 0):
                yy -= 1
        LCD.pixel(xx+15,yy+(i%5)-8,bg)
    yy = y
    #p2
    for i in range(40):
        if(i % 5 == 0):
            xx -= 1
            if(i % 10 == 0):
                yy -= 1
        LCD.pixel(xx+15,yy+(i%5)-12,bg)
        
    #back side of seven
    for i in range(40):
        if(i % 5 == 0):
            xx -= 1
            if(i % 10 == 0):
                yy -= 1
        LCD.pixel(xx+24,yy+(i%5)+3,bg)

def numberEight(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of the eight
    for i in range(25):
        if(i % 5 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+16,yy+(i%5)-8,bg)
    #bg at bottom of the eight
    for i in range(25):
        if(i % 5 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+13,yy+(i%5)-8,bg)


def numberNine(x,y,c,bg):
    xx = x
    yy = y
    #creates a block
    for i in range(209):
        if(i % 19 == 0):
            yy -= 1
            xx = x
        LCD.pixel(xx+(i%19),yy,c)
    #bg at top of nine
    for i in range(28):
        if(i % 7 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+7,yy+(i%7)-11,bg)
    #bg in middle of nine
    for i in range(16):
        if(i % 4 == 0):
            xx -= 1
            yy = y
        LCD.pixel(xx+19,yy+(i%4)-8,bg)


number_list = [numberZero, numberOne, numberTwo, numberThree,
               numberFour, numberFive, numberSix, numberSeven, numberEight, numberNine]



#=================Screen init=================

def sin_lines(point, amount, radius):
    return int(radius * math.sin(point * (2 * math.pi)/amount))

def cos_lines(point, amount, radius):
  return int(radius * math.cos(point *(2 * math.pi)/amount))

# Calculate end of hand offsets
def end_point(angle, radius): 
    ang_to_rad = math.radians(angle)
    x_cord = int(radius * math.sin(ang_to_rad))
    y_cord = -int(radius * math.cos(ang_to_rad))                     
    return x_cord,y_cord
#=================End screen init=================

#=================colors=================

cool_blue = colour(71, 99, 255)
something = colour(255, 174, 120)
green = colour(63, 255, 5)
red = colour(255,0,0)
m_org = colour(245, 129, 27)
c_gray = colour(168, 162, 157)
d_gray = colour(79, 77, 75)
white = colour(255,255,255)
black = colour(0,0,0)

#=================init time=================
hour = 3
minute = 15
second = 15
#=================End init time=================
button1 = machine.Pin(17, machine.Pin.IN, machine.Pin.PULL_UP)
button2 = machine.Pin(18, machine.Pin.IN, machine.Pin.PULL_UP)

def starter_dial():
    circle(120,120,121,m_org)
    for p in range(0,360,6):
        hxn, hyn = end_point(p, 120)
        LCD.line(120,120,120+hxn,120+hyn,colour(0,0,0))


circle(119,120,108,colour(0,0,0))

#Defines what screen is shown
display_setting = 0
#Counter for how long the screens been on
counter = 0
#full batt charge
battery_level = 8



# Flag to track LCD state
lcd_on = False

#Reads gyrodata to check if should I turn on
def wake_up():
    xyz=qmi8658.Read_XYZ()
  
    if(abs(xyz[3]) > 140 or abs(xyz[4]) > 140):
        lcd_on = True
    else:
        lcd_on = True
    
    return lcd_on


starter_dial()
##=================MAIN CODE=================
while(True):
    #Checks to see if I should turn on
    lcd_on = wake_up()
    
    if(not lcd_on and counter == 0):
        LCD.turn_off()
    
    if (counter == 15):
        counter = 0
        
    #button one is pressed
    if(button1.value() == 0 and button2.value() == 1):
        #move setting up one
        display_setting += 1
        #reset on timer
        counter = 1
        #resets setting if goes to far
        if(display_setting == 3):
            starter_dial()
            display_setting = 0
    #button two is pressed
    elif(button2.value() == 0 and button1.value() == 1):
        #resets timer
        counter = 1
        #moves back a setting
        display_setting -= 1
        if(display_setting == 0):
            starter_dial()
        elif(display_setting == -1):
            display_setting = 0
    #Both are pressed in main loop
    elif(button1.value() == 0 and button2.value() == 0):
        #resets battery level
        battery_level = 8
    
    
    #===Keeps track of time in the back ground====
    
    second += 1
    
    if(lcd_on or counter > 0):
        counter += 1
        
    #Checks to see if a minute has passed
    if(second >= 75):
        #Resets second timer
        second = 15
        #Adds a minute
        minute += 1

    #Checks to see if an hour has passed
    if(minute >= 75):
        #resets mintue timer
        minute = 15
        #Adss an hour
        hour += 1
        
        battery_level -= 1
    #Checks to see if 12 have passed
    if(hour >= 26):
        #resets hour hand
        hour = 3

    #=========End time========
    

    
    #Clock Face
    if (display_setting == 0):
        
        circle(119,120,108,colour(0,0,0))
        
        #numbers on the dial
        numberOne(200,120 ,colour(255,255,255),black)
        numberTwo(200,135,colour(255,255,255),black)
        numberThree(111,223,colour(255,255,255),black)
        numberSix(17,126,colour(255,255,255),colour(0,0,0))
        numberNine(111,30,colour(255,255,255),colour(0,0,0))
        
        
        #End cords for second line
        sx, sy = end_point(second*6, 106)
        #end cords for minute line
        mx, my = end_point(minute*6, 95)
        #end cords for hour line
        hx, hy = end_point(hour*30, 70)
        
        #Draws 12 on the dial
        #display.text(font,"12",105,20,fg,bg)
            
        #Draws second line
        LCD.line(120,120,118+sx,120+sy,m_org)
        #Draws minute line
        #LCD.line(120,120,118+mx,120+my,red)
        mxnsm = int(mx/15)
        mynsm = int(my/15)    
        tri_filled(120+mx, 120+my, 120+mynsm, 120-mxnsm, 120-mynsm, 120+mxnsm,c_gray)
        #Draws hour line
        #LCD.line(120,120,118+hx,120+hy,green)
        hxnsm = int(hx/8)
        hynsm = int(hy/8)
        tri_filled(120+hx, 120+hy, 120+hynsm, 120-hxnsm, 120-hynsm, 120+hxnsm,d_gray)
       
        #circle in the middle
        circle(119,120,13,d_gray)
        circle(119,120,11,m_org)
        circle(119,120,9,d_gray)

    if(display_setting == 1):
        circle(119,120,125,colour(0,0,0))
        
        #Empty battery symbol
        #back line
        LCD.line(160,50,80,50,colour(255,255,255))
        LCD.line(160,51,80,51,colour(255,255,255))
        #top line
        LCD.line(160,50,160,180,colour(255,255,255))
        LCD.line(161,50,161,180,colour(255,255,255))
        #bottom line
        LCD.line(80,50,80,180,colour(255,255,255))
        LCD.line(81,50,81,180,colour(255,255,255))
        #end line
        LCD.line(160,180,80,180,colour(255,255,255))
        LCD.line(160,181,80,181,colour(255,255,255))
        #button
        for i in range(1,6):
            LCD.line(135-i,180+i,105+i,180+i,colour(255,255,255))
            
        #Decides battery level
        if(battery_level > 6):
            bat_ticker = 1
            while(bat_ticker < 129):
                LCD.line(159,51+bat_ticker,82,51+bat_ticker,colour(0,255,0))
                bat_ticker += 1
        elif(battery_level > 4):
            bat_ticker = 1
            while(bat_ticker < 97):
                LCD.line(159,51+bat_ticker,82,51+bat_ticker,colour(0,255,0))
                bat_ticker += 1
        elif(battery_level > 2):
            bat_ticker = 1
            while(bat_ticker < 65):
                LCD.line(159,51+bat_ticker,82,51+bat_ticker,colour(0,255,0))
                bat_ticker += 1
        else:
            bat_ticker = 1
            while(bat_ticker < 33):
                LCD.line(159,51+bat_ticker,82,51+bat_ticker,colour(255,0,0))
                bat_ticker += 1
        

    if(display_setting == 2):
        
        hour_picker = 1
        minute_picker = 1
        which = True
        
        while(True):
            #Test to break out
            if(button1.value() == 0 and button2.value() == 0):
                break
            elif(button1.value() == 0 and which == True):
                minute_picker += 1
                if(minute_picker == 60):
                    minute_picker = 0
            elif(button1.value() == 0 and which == False):
                hour_picker += 1
                if(hour_picker == 13):
                    hour_picker = 1
            elif(button2.value() == 0):
                which = not(which)

                
            circle(119,120,240,colour(0,0,0))
            
            if(not which):
                LCD.line(113,100,113,83,m_org)
                LCD.line(114,100,114,83,m_org)
            else:
                LCD.line(113,110,113,140,m_org)
                LCD.line(114,110,114,140,m_org)
            
            
            xx = 120
            yy = 112
            for i in range(25):
                if(i % 5 == 0):
                    yy -= 1
                    xx = 120
                LCD.pixel(xx+(i%5),yy,white)
                
            xx = 130
            yy = 112
            for i in range(25):
                if(i % 5 == 0):
                    yy -= 1
                    xx = 130
                LCD.pixel(xx+(i%5),yy,white)
            
            h = str(hour_picker)
            if(len(h) == 1):
                number_list[hour_picker](120,105,white,black)
            else:
                number_list[int(h[1])](120,100,white,black)
                number_list[int(h[0])](120,88,white,black)
            
            
            if(minute_picker == 0):
                number_list[0](120,120,white,black)
                number_list[0](120,130,white,black)
            else:
                p = str(minute_picker)
                if(len(p) == 1):
                    number_list[0](120,120,white,black)
                    number_list[minute_picker](120,140,white,black)
                else:
                    p1 = p[0]
                    p2 = p[1]
                    number_list[int(p1)](120,125,white,black)
                    number_list[int(p2)](120,140,white,black)
                    
            
            time.sleep(0.09)
            LCD.show()
            
        #Outside the loop
        hour = hour_picker+3
        minute = minute_picker+15
        second = 0
        display_setting = 0
        starter_dial()

    #sleeps to keep acurate time
    time.sleep(0.9093334)
    
    #test to see if display should show or not
    if(lcd_on == True and counter == 1):
        LCD.init_display()
        display_setting = 0
    if(counter > 0):
        LCD.show()


    








