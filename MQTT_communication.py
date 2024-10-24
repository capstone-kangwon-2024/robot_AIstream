import paho.mqtt.client as paho
import smbus  # i2c 통신을 위해
from datetime import datetime
import time
import signal
import numpy as np
import cv2
import os
import atexit
import subprocess

# 개발하면서 생성한 로봇 조종명령어 목록
cmd_list = {
    "Forward":   "F",
    "Backward":  "B",
    "Left":      "L",
    "Right":     "R",
    "Stop":      "S",
    "Fire":      "FIRE", 
    "ForceFire": "FFIRE",
    "RC":        "RC",
    "Feed":      "Feed",
    "ChangeMode": "M",
    "AutoDtt":    "AD",
    "AutoDttF":   "ADF",
    "F1":         "F1",
    "I2C":        "."
}


def checkRunningProcess():
    cmd = "ps -ef"
    
    res = subprocess.run(cmd.split(' '), stdout=subprocess.PIPE, text=True)
    res_lines = res.stdout.splitlines()

    cnt = -1
    for str_ in res_lines:
        if str_.find("Capstone") != -1: cnt+=1
        if cnt == 1: exit()

def writeData(value):
    byte_str = StringToBytes(value)
    
    try:
        nano.write_i2c_block_data(nano_addr, ord('1'), byte_str)
    except OSError:
        print("I2C comm failed. check nano")
        client.publish('respone', "I2C comm failed. check nano", qos=2)
        pass
        
    return -1

def StringToBytes(val):
        retVal = []
        for c in val: retVal.append(ord(c))
        return retVal

def on_connect(client, userdata, flags, rc, properties=None):
    print("CONNACK received with code %s." % rc)
    cur_time = time.strftime('%H:%M:%S')
    
    conn_respone = cur_time + " --> Online!"
    client.publish('respone', conn_respone, qos=0)
    writeData("SPD180170")
    
def on_disconnect(client, userdata, rc, properties):
    print("DISCONN received with code %s." % rc)
    cur_time = time.strftime('%H:%M:%S')
    
    conn_respone = cur_time + " --> Offline~"
    client.publish('respone', conn_respone, qos=0)

def on_subscribe(client, userdata, mid, granted_qos, properties=None):
    print("Subscribed: " + str(mid) + " " + str(granted_qos[0]))

def on_message(client, userdata, msg):
    payload = msg.payload.decode('utf-8') # MQTT 수신 msg는 bytes형이므로 str형으로 변환
    
    if msg.topic == "check":
        if payload == "alive": 
            client.publish('respone', "yes", qos=2)
        elif payload == "time":  
            cur_time = time.strftime('%H:%M:%S')
            cur_time += " %sms" % str(datetime.utcnow().microsecond)[0:3]
            client.publish('respone', cur_time, qos=0)
        elif payload == "test":
            try:
                raw = nano.read_i2c_block_data(0x8, ord('0'), 16)
                for c in raw:
                    if c != 255: respone += chr(c)
                    
                print("recived msg = %s" % respone)
                respone = ""
            except OSError:
                print("I2C comm failed. check nano")
                client.publish('respone', "I2C comm failed. check nano", qos=2)
                pass
        elif payload == "rst_stm":
            global ai_stream_py
            
            print(ai_stream_py.poll())
            if ai_stream_py.poll() == None:
                ai_stream_py.terminate()
                ai_stream_py.wait(timeout=None)
            else:
                ai_stream_py = subprocess.Popen(args=['python3', '4proto.py'])
        elif payload == "rst":
            handler(None, None)
    elif msg.topic == "control":
        if payload not in list(cmd_list.keys()):  # 정의한 메시지가 아닐 시 오류.
            print("wrong payload of control.")
        else:
            if payload == "I2C": nano = smbus.SMBus(1)  # I2C통신 다시 시작.
            writeData(cmd_list[payload])
            
    print("%s : %s" % (msg.topic, payload))  
    
def exit_handler(signum, frame):
    print("KeyboardInterrupt recived!!!")
    
    cur_time = time.strftime('%H:%M:%S')
    
    conn_respone = cur_time + " --> Offline~"
    client.publish('respone', conn_respone, qos=0)
    
    ai_stream_py.terminate()
    ai_stream_py.wait(timeout=None)
    
    client.disconnect()  # 서버 연결해제
    
    exit()

# 프로그램 종료시 종료절차를 수행하도록 함
signal.signal(signal.SIGINT, exit_handler)  

# 중복되어 실행되는 것을 방지
checkRunningProcess()
ai_stream_py = subprocess.Popen(args=['python3', '4proto.py'])

# 실제 조종을 담당하는 컴퓨터와 통신을 시작. 통신방식은 I2C.
nano = smbus.SMBus(1)
nano_addr = 0x8
flag_connected = 0

# 설치해둔 MQTT서버에 접속
client = paho.Client(client_id="gil-jetnano", userdata=None, protocol=paho.MQTTv5)
client.tls_set(tls_version=paho.ssl.PROTOCOL_TLS) # 접속 시 TLS보안기능을 적용합니다

client.on_connect    = on_connect
client.on_disconnect = on_disconnect
client.on_subscribe  = on_subscribe
client.on_message    = on_message

client.username_pw_set('아이디', '비밀번호') # 권한을 부여한 아이디와 비밀번호를 입력
client.connect('1c15066522914e618d37acbb80809524.s1.eu.hivemq.cloud', 8883)

# 총 2가지 명령을 수신받도록 설정
# 1. 로봇상태 확인 및 관리 명령 [ check ]
# 2. 로봇 조종명령 [ control ]
client.subscribe('check', qos=2) 
client.subscribe('control', qos=0)

client.loop_forever()