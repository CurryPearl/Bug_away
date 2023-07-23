#霍尔编码器AB相测速+PWM脉宽调制

import sensor, image, time,pyb,struct,math
from pyb import UART, ADC
from machine import I2C,Pin
from ssd1306x import SSD1306_I2C
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
r1=0
r2=10
r1_set=0
r2_set=0
x=-1
y=-1
def drawline(x1,y1,x2,y2):
 global dir1,dir2,r1,r2,r1_set,r2_set,x,y
 L=10
 if abs(x1-x2)>0.1:
  if x==-1:
   x = min(x1,x2)  # 变量x初始值设为x1
  r1_set=0
  r2_set=0
  if x != max(x1,x2):  # 当x不等于x2时进行循环
     delta_x = abs(x2 - x1)  # 计算|x2 - x1|
     x += delta_x * 0.001  # 每次循环加上千分之一的|x2 - x1|
     k = (y2 - y1) / (x2 - x1)
     b = y1 - k * x1
     r1_set=math.sqrt((k**2 + 1) * x**2 + 2 * k * b * x + b**2)
     r2_set=math.sqrt((x-L)**2+k**2*x**2+ 2 * k * b * x + b**2)
  else:
   x=-2
 else:
  if y==-1:
   y=min(y1,y2)
  r1_set=0
  r2_set=0
  if y != max(y1,y2):  # 当x不等于x2时进行循环
     delta_y = abs(y2 - y1)  # 计算|x2 - x1|
     y += delta_y * 0.001  # 每次循环加上千分之一的|x2 - x1|
     r1_set=math.sqrt(x1**2+y**2)
     r2_set=math.sqrt((x1-L)**2+y**2)
  else:
   y=-1
 if(r1_set-r1>0):
       dir1=1
 else:
       dir1=-1
 if(r2_set-r2>0):
       dir2=1
 else:
       dir2=-1

dir1=1
dir2=1
button1=0
button2=0
button3=0
button4=0
button5=0
zzz=0
adc_p = ADC("P6")
adc_p.read()  #读取一次以激活ADC
angle1 = 0
angle2 = 0
page=1
sw1_flag=0
sw2_flag=0
sw3_flag=0
sw4_flag=0
sw5_flag=0
line_flag=0
i2c = I2C(sda=Pin("P5"), scl=Pin("P4"),freq=80000 )#这里按照openmv的引脚进行初始化i2c对象
oled = SSD1306_I2C(128, 64, i2c, addr=0x3c)#根据该i2c对象生成oled对象
def inter(t):
 global button1,button2,button3,button4,button5,adc_p,zzz,sw1_flag,sw2_flag,sw3_flag,sw4_flag,sw5_flag,page,dir1,dir2,oled,r1,r2,line_flag
 if zzz<1000000:
     zzz+=1
 if zzz%5==0:
    button = adc_p.read()  # 读取ADC转换后的值
    if button>3100 and button<3150:
        button1=1
    elif button>3250 and button<3300:
        button2=1
    elif button>3300 and button<3400:
        button3=1
    elif button>3450 and button<3600:
        button4=1
    elif button>3700 and button<3850:
        button5=1
    else:
        button1=0
        button2=0
        button3=0
        button4=0
        button5=0
     #---------------------------#
    if(button1==1):
     sw1_flag=1
    elif sw1_flag==1:
     sw1_flag=0
     if page<9:
      page+=1
     else: page=1
    #---------------------------#
    #第一页
    if(page==1):
     if(button2==1):
      sw2_flag=1
     elif sw2_flag==1:
      sw2_flag=0
      dir1=-1

     if(button3==1):
      sw3_flag=1
     elif sw3_flag==1:
      sw3_flag=0
      dir1=1

     if(button4==1):
      sw4_flag=1
     elif sw4_flag==1:
      sw4_flag=0
      dir2=-1
     if(button5==1):
      sw5_flag=1
     elif sw5_flag==1:
      sw5_flag=0
      dir2=1
 if zzz%15==0:
   line_flag=1

#####################################################################
timer=pyb.Timer(4,freq=1000,callback=inter)
timer.channel(1,pyb.Timer.PWM,pin=pyb.Pin("P8"),pulse_width_percent=0)
timer.channel(2,pyb.Timer.PWM,pin=pyb.Pin("P7"),pulse_width_percent=0)#初始化PWM口
#####################################################################
encoder_pin_a = pyb.Pin("P3", pyb.Pin.IN)
encoder_pin_b = pyb.Pin("P9", pyb.Pin.IN)
uart = UART(1,9600)   #定义串口3变量
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters
def callback1(line):
    global angle1,r1
    if(dir1==1):
      angle1 += 1#每旋转一定角度，会因为下降沿进入该中断
    elif dir1==-1:
       angle1 -=1
    r1=angle1/100
def callback2(line):
    global angle2,r2
    if(dir2==1):
      angle2 += 1#每旋转一定角度，会因为下降沿进入该中断
    elif dir2==-1:
       angle2 -=1
    r2=angle2/100

extint_1 = pyb.ExtInt(encoder_pin_a, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_UP, callback1)
extint_2 = pyb.ExtInt(encoder_pin_b, pyb.ExtInt.IRQ_RISING, pyb.Pin.PULL_UP, callback2)

#设置IO口外部触发中断，设置为下降沿触发，内部设置为上拉电阻，设置中断回调函数为callback
########################################################################################

timer.channel(1).pulse_width_percent(40)#占空比50%
timer.channel(2).pulse_width_percent(40)#占空比50%
while(True):
    clock.tick()
    img = sensor.snapshot()
    if uart.any():
        a = uart.read(1)            #uart.read()为一个字节
        b=a[0]%16
        print(b)
    #uart.write("sb0")
   # print(angle1)
   # print(angle2)
    if(line_flag==1):
     line_flag==0
     #drawline(0,0,20,20)
     if dir1==1 and dir2==1:
       uart.write("sb0")
     elif dir1==1 and dir2==-1:
       uart.write("sb1")
     elif dir1==-1 and dir2==1:
       uart.write("sb2")
     elif dir1==-1 and dir2==-1:
       uart.write("sb3")

    #---------------------------#
    oled.fill(0)#清屏
    if(page==1):
     oled.text("r1:",0,0)#字符显示函数-常量
     oled.text(str(int(r1)),56,0)#字符显示函数-变量
     oled.text("r2:",0,8)#字符显示函数-常量
     oled.text(str(int(r2)),56,8)#字符显示函数-变量
     oled.text("r1_set:",0,16)#字符显示函数-常量
     oled.text(str(int(r1_set)),56,16)#字符显示函数-变量
     oled.text("r2_set:",0,24)#字符显示函数-常量
     oled.text(str(int(r2_set)),56,24)#字符显示函数-变量
    #oled.text("SW1:",0,16)#字符显示函数-常量
    #oled.text(str(button1),56,16)#字符显示函数-变量
    #oled.text("SW2:",0,24)#字符显示函数-常量
    #oled.text(str(button2),56,24)#字符显示函数-变量
    #oled.text("SW3:",0,32)#字符显示函数-常量
    #oled.text(str(button3),56,32)#字符显示函数-变量
    #oled.text("SW4:",0,40)#字符显示函数-常量
    #oled.text(str(button4),56,40)#字符显示函数-变量
    #oled.text("SW5:",0,48)#字符显示函数-常量
    #oled.text(str(button5),56,48)#字符显示函数-变量
    oled.text(str(page),120,56)#字符显示函数-变量
    oled.show()#显示（不执行该条的话，上面的text只存于缓冲区）
    #print(button," ",button1," ",button2," ",button3," ",button4," ",button5," ")








