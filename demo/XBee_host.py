import serial
import time

#--------------------------------XBee setting---------------------------------#
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())
s.write("ATMY 0x130\r\n".encode())
char = s.read(3)
print("Set MY 0x130.")
print(char.decode())
s.write("ATDL 0x230\r\n".encode())
char = s.read(3)
print("Set DL 0x230.")
print(char.decode())
s.write("ATID 0x1\r\n".encode())
char = s.read(3)
print("Set PAN ID 0x1.")
print(char.decode())
s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())
s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())
s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())
s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())
#-----------------------------------------------------------------------------#

print("start sending RPC")                  # 開始傳送RPC指令

while True:                                 # 無窮迴圈
    s.write("/getAcc/run\n\r".encode())     # 傳送自定義RPC指令，取板子加速度
    char = s.read(200)                      # 讀板子回覆
    print(char.decode())                    # 顯示回覆
    time.sleep(1)                           # 停頓一秒

s.close()                                   # 結束