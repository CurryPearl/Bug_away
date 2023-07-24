import sensor, image, time,pyb,struct,math
from pyb import UART, ADC, LED
from machine import I2C,Pin
from pid import PID


sensor.reset()
sensor.set_vflip(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQQVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
LED(1).on()
LED(2).on()
LED(3).on()
clk = pyb.Pin("P2",Pin.OUT)
Si = pyb.Pin("P3",Pin.OUT)
adc_p = ADC("P6")
ADV = [0] * 128
adc_p.read()  #读取一次以激活ADC
zzz=0
zzz_remember=-1500

flag_rec=0
i=-1
c=[0,0,0,0,0,0,0,0,0,0]

##以下为全局变量
cha=0
_threshold=0        #每次的阈值
count_start=15      #计算谷底时的起始点位
count=0             #down_array中寄存的谷底个数
down_start=0        #黑线开始坐标
down_end=0          #黑线结束坐标
max_val=0           #数组中最大值
min_val=0           #数组中最小值
flag=0              #zzz写的
remember=1          #某个记忆值
average=0           #数组均值
guan_dian_count=0   #拐点计数
CROSS_FLAG=0        #用于判断走内圈还是外圈，1为外圈，0为内圈
WAY_ORDER=0         #用于判断顺时针还是逆时针，1为逆时针，0为顺时针
down_array=[0,0,0,0]#定义全局列表，用于存放凹陷处的中点坐标，最多出现四个凹陷（拐点2+误判1/2）

#def uart_read():
    #if uart.any():
        #data = uart.read(1)  # 读取一个字节的数据
        #a=data[0]%16
        #if(a==10):
          #flag_rec=1
          #i=-1
          #for j in range(0,10):
             #c[j] = 0
        #elif flag_rec==1 and i<9:
          #i+=1
          #c[i]=a
          #if i==9:
           #i=-1
           #flag_rec=0
#def parameter_clear():
    #uart_read()
    #if c[3]==1:
        #cha=0
        #_threshold=0        #每次的阈值
        #count_start=15      #计算谷底时的起始点位
        #count=0             #down_array中寄存的谷底个数
        #down_start=0        #黑线开始坐标
        #down_end=0          #黑线结束坐标
        #max_val=0           #数组中最大值
        #min_val=0           #数组中最小值
        #flag=0              #zzz写的
        #remember=1          #某个记忆值
        #average=0           #数组均值
        #guan_dian_count=0   #拐点计数
        #CROSS_FLAG=0        #用于判断走内圈还是外圈，1为外圈，0为内圈
        #WAY_ORDER=0         #用于判断顺时针还是逆时针，1为逆时针，0为顺时针
        #down_array=[0,0,0,0]#定义全局列表，用于存放凹陷处的中点坐标，最多出现四个凹陷（拐点2+误判1/2）


def inter(t):
 global button1,button2,button3,button4,button5,adc_p,zzz,sw1_flag,sw2_flag,sw3_flag,sw4_flag,sw5_flag,page,oled
 if zzz<10000000:
     zzz+=1
timer=pyb.Timer(4,freq=1000,callback=inter)

def ccd_get():
    global ADV
    i = 0
    tslp = 0
    clk.high()
    pyb.udelay(20)
    Si.low()
    pyb.udelay(20)
    Si.high()
    pyb.udelay(20)
    clk.low()
    pyb.udelay(20)
    clk.high()
    pyb.udelay(20)
    Si.low()
    pyb.udelay(20)

    for i in range(128):
        clk.low()
        pyb.udelay(40)

        ADV[tslp] = adc_p.read()>> 3
        if tslp==85:
            ADV[tslp]+=20
        elif tslp==86:
            ADV[tslp]+=20
        elif tslp==87:
            ADV[tslp]+=20
        tslp += 1
        clk.high()
        pyb.udelay(40)

def down_mid():#传回凹陷的中点坐标，如果不存在凹陷，则返回0
    global _threshold
    global count_start
    global down_start
    global down_end     #声明全局变量
    remember=1          #用于使得down_start只计数一次
    for i in range(count_start,115):
        if  ADV[i]<_threshold:
            if(remember):
                down_start=i
                remember=0
        elif ADV[i]>_threshold and remember==0:
            down_end=i
            count_start=i
            break
    if remember==1:
        return 0
    else:
        return int((down_start+down_end)*0.5)

'''这个函数不需要
def filter_down():#如果滤除尖刺，返回1，反之返回0
    if abs(down_end-down_start)<=4:
        for i in range(down_start,down_end):
            ADV[i]=max_val
            return True
    return False
'''

#以下函数用于寻找中点，包含拐点判断
def search_mid(cross_flag,way_order):#当cross_flag为1时走外圈，为0时走内圈;当way_order为1时逆时针，为0时顺时针
    global _threshold
    global count
    global count_start#记录凹陷起点的标志位置0
    global down_start
    global down_end
    global max_val
    global min_val
    global average
    global down_array
    global flag
    global guan_dian_count
    global zzz          #用于计时，每次1毫秒
    global zzz_remember #用于存储上一次的计数值，用于屏蔽拐点计数
    min_val=ADV[16]
    max_val=ADV[16]
    count=0
    count_start=15
    down_start=15
    down_end=15
    average=0           #变量初始化
    _threshold_remember=_threshold
    down_array_remember=down_array  #如果冲出赛道，用于记录上一次的值
    for i in range(15,113):#找出最大最小值，注意，这里没有滤除特殊值，但是经过检验影响不大
        if ADV[i]<min_val:
            min_val=ADV[i]
        if ADV[i]>max_val:
            max_val=ADV[i]
    list_sort=ADV[16:112]
    list_sort.sort()#列表排序
    for i in range(0,86):   #排序完之后剔除10个最大值，以避免突发高刺的影响
        average+=list_sort[i]/86.0
    _threshold = int((average ) / 2.0)  # 计算阈值
    if abs(max_val-_threshold)<25:  #经过检测，当冲出赛道时，最大值和阈值的差值不会超过25，在15左右浮动，故可以如此检测
        down_array=down_array_remember
    else:
        for i in range(0,4):
            if count_start>115:
                break
            else:
                down_array[i]=down_mid()
    for i in range(0,4):
        if(down_array[i]!=0):
            count+=1 #判断当前是否遇到拐点
    if count==2:#只需要判断count==2的情况，其他情况全部归于else即可
        if zzz-zzz_remember>1500:######注意这里的1500需要根据速度实时调整
            guan_dian_count+=1
            zzz_remember=zzz
        #以下为根据CROSS_FLAG和WAY_ORDER的值，来选择走内圈还是外圈
        if way_order==1:
            if cross_flag==1:
                for i in range(3,-1,-1):
                    if down_array[i]!=0:
                        break
                return down_array[i]#逆时针，走外圈时，回传第二个凹谷的中间值
            elif cross_flag==0:
                for i in range(0,4):
                    if down_array[i]!=0:
                        break
                return down_array[i]#逆时针，走内圈时，回传第一个凹谷的中间值
        elif way_order==0:
            if cross_flag==1:
                for i in range(0,4):
                    if down_array[i]!=0:
                        break
                return down_array[i]#顺时针，走外圈时，回传第一个凹谷的中间值
            elif cross_flag==0:
                for i in range(3,-1,-1):
                    if down_array[i]!=0:
                        break
                return down_array[i]#顺时针，走内圈时，回传第二个凹谷的中间值
    else:
        for i in range(0,4):
            if down_array[i]!=0:
                break
        return int(down_array[i])

def sendtopc():
    uart.write("*")
    uart.write("LD0000")
    for i in range(0, 128):
        num=ADV[i]
        high_nibble = (num >> 4) & 0xF   # 获取高四位
        low_nibble = num & 0xF           # 获取低四位

        high_hex = '%X' % high_nibble    # 转换为对应字符（大写）
        low_hex = '%X' % low_nibble      # 转换为对应字符（大写）


        uart.write(high_hex)  # 发送高八位对应的字符
        uart.write(low_hex)   # 发送低八位对应的字符


    uart.write("00")
    uart.write("#")
######
uart = UART(1,9600)
uart.init(9600, bits=8, parity=None, stop=1)


while(True):
    clock.tick()
    ccd_get()
    pyb.udelay(20)
    ccd_get()
    print(guan_dian_count)
    print(down_array)
    #当cross_flag为1时走外圈，为0时走内圈;当way_order为1时逆时针，为0时顺时针
    cha=int(0.5*(search_mid(1,1)-64))

    if(cha>30):
     cha=30
    elif(cha<-30):
     cha=-30
    uart.write('s')
    if(cha>=0):
     uart.write('00')
    else:
     uart.write('01')
    str_num=str(abs(int(cha)))
    if abs(cha) > 9:
     uart.write(str_num)  # 将整数转换为字节型数据并发送
    else:
     uart.write('0')  # 发送字节型数据 '0'
     uart.write(str_num)  # 将整数转换为字节型数据并发送
    uart.write('00')
    uart.write('0%d'%(guan_dian_count))
    uart.write('00')
    #parameter_clear()
    #sendtopc()
    #abcded
