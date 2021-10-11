#
# This software is for ISP Remote Node.
#
import socket
import utime
import ubinascii
import struct
import pycom
import machine
import ssl
from machine import RTC, UART, Pin
import os
from network import LoRa, WLAN
from parametros import *
from udht.dht import DTH
from uModBus.serial import Serial
import uModBus.meatrolcons as Meatrolcons
from umqtt.robust import MQTTClient
from uping.ping import ping


##### Define Configuration #####

### Start Pybytes
from _pybytes import Pybytes
from _pybytes_config import PybytesConfig
conf = PybytesConfig().read_config()
pybytes = Pybytes(conf)

pybytes.start()

### Board ID
client_id = ubinascii.hexlify(machine.unique_id())
print('Cliente ID:', client_id)

### UDHT
dht1 = DTH(Pin('P23', mode=Pin.OPEN_DRAIN),0)

### uModBus Configuration
modbus_obj = Serial(1, baudrate=9600, data_bits=8, stop_bits=1,
                    parity=None, pins=('P3', 'P4'), ctrl_pin=('P8'))

# Tuplas for the some registres form Meatrol Power Meter
SLAVE_ADDR = 01
SIGNED = True
REG_MODBUS = [2147, 2149, 2151, 2139, 2141, 2143, 2000, 2002, 2004, 2155, 2157, 2159,
              2163, 2165, 2167, 2171, 2173, 2175, 4016, 4018, 4020, 4040, 4042, 4044, 4064, 4066, 4068]
REG_CHANN = [0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x0D, 0x0E, 0x0F, 0x11, 0x12, 0x13, 0x15,
             0x16, 0x17, 0x19, 0x1A, 0x1B, 0x1D, 0x1E, 0x1F, 0x21, 0x22, 0x23, 0x25, 0x26, 0x27]
REG_SIZE = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
            2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]
REG_TYPE = [0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03,
            0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03]
REG_VALUE = [0, 0, 0, 0, 0, 0]
ENERGY_VALUE = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

### LORAWAN Configuration
# TTN Parameters
dev_addr = struct.unpack(">l", ubinascii.unhexlify(
    '26 0C 33 BF'.replace(' ', '')))[0]
nwk_swkey = ubinascii.unhexlify(
    '61452F4930E22264593317A0D9299DD8'.replace(' ', ''))
app_swkey = ubinascii.unhexlify(
    '59BEEE9654FEF8FD497454D3B6852410'.replace(' ', ''))

#Lora Wan Type
lora = LoRa(mode=LoRa.LORAWAN)

# Band for the Gateway
def select_subband(lora, subband):
    if (type(subband) is int):
        if ((subband < 1) or (subband > 8)):
            raise ValueError("subband out of range (1-8)")
    else:
        raise TypeError("subband must be 1-8")
    for channel in range(0, 71):
        lora.remove_channel(channel)
    for channel in range((subband-1)*8, ((subband-1)*8)+8):
        lora.add_channel(channel, frequency=902300000
                         + channel*200000, dr_min=0, dr_max=3)
        print(channel)

sb = 1
select_subband(lora, sb)

s = socket.socket(socket.AF_LORA, socket.SOCK_RAW)
s.setblocking(False)
s.setsockopt(socket.SOL_LORA, socket.SO_DR, 3)

# Gateway and TTN Conection
lora.join(activation=LoRa.ABP, auth=(dev_addr, nwk_swkey, app_swkey))
utime.sleep(5)

x = 0
waitjoin = 0
while lora.has_joined() == False and waitjoin < 20:
    print('Not joined yet...')
    pycom.heartbeat(False)
    pycom.rgbled(0x7f0000)
    time.sleep(0.1)
    pycom.rgbled(0x000000)
    utime.sleep(2)
    waitjoin = waitjoin+1

### Time RTC Configuration
rtc = RTC()
rtc.ntp_sync(ntp_host, 3600, ntp_backup)
print("Hora Actual: ", rtc.now())

### MQTT Ubidots Topic Configuration
#Topic MQTT MCU
topic_pub_rssi = b'/v1.6/devices/'+client_id+'/rssi'
topic_pub_internet = b'/v1.6/devices/'+client_id+'/internet'
topic_pub_u1 = b'/v1.6/devices/'+client_id+'/u1'
topic_pub_u2 = b'/v1.6/devices/'+client_id+'/u2'
topic_pub_u3 = b'/v1.6/devices/'+client_id+'/u3'
topic_pub_i1 = b'/v1.6/devices/'+client_id+'/i1'
topic_pub_i2 = b'/v1.6/devices/'+client_id+'/i2'
topic_pub_i3 = b'/v1.6/devices/'+client_id+'/i3'
topic_pub_temp = b'/v1.6/devices/'+client_id+'/temp'
topic_pub_hum = b'/v1.6/devices/'+client_id+'/hum'
topic_pub_alm01 = b'/v1.6/devices/'+client_id+'/alm01'
topic_pub_alm02 = b'/v1.6/devices/'+client_id+'/alm02'
topic_pub_alm03 = b'/v1.6/devices/'+client_id+'/alm03'
topic_pub_alm04 = b'/v1.6/devices/'+client_id+'/alm04'
topic_pub_alm05 = b'/v1.6/devices/'+client_id+'/alm05'

TOPIC_VALUE = [topic_pub_u1, topic_pub_u2, topic_pub_u3,
               topic_pub_i1, topic_pub_i2, topic_pub_i3]

#Topic Funcionales
topic_pub_error = b'/v1.6/devices/'+client_id+'/error'

#Topic Subscribe
topic_sub_tsgral = b'/v1.6/devices/'+client_id+'/tsgral'

#####  Fuctions Define #####

def callback(topic, msg):
    global tsgral
    print((topic, msg))
    if topic == topic_sub_tsgral:
        tsgral = float(msg[9:12])
        print('Time Sleep Frequency:', tsgral)


def connect_mqtt():
    global client_id, mqtt_host, mqtt_user
    client = MQTTClient(client_id, mqtt_host, port=8883, user=mqtt_user, password=mqtt_pass, ssl=True, ssl_params={
                        'cert_reqs': ssl.CERT_REQUIRED, 'ca_certs': '/flash/cert/industrial.pem'})
    client.set_callback(callback)
    print('Tratando de conectar al Servidor MQTT')
    client.connect()
    print('Subscribo Variables')
    client.subscribe(topic_sub_tsgral, qos=0)
    print('Connected to %s MQTT broker' % (mqtt_host))
    utime.sleep(1)
    return client


def lectura():
    print('*****************************************')
    sec = 'Lectura: ' + "{:02d}/{:02d}/{}".format(rtc.now()[2], rtc.now()[1], rtc.now(
    )[0]) + " {:02d}:{:02d}:{:02d}".format(rtc.now()[3], rtc.now()[4], rtc.now()[5])
    print(sec)
    print('*****************************************')


def report_error(error_code=0):
    # IN: str error = texto de error
    try:
        f = open('errores.txt', "a")
        # si el archivo existe, continúe
    except OSError:  # no se abre, creando uno nuevo
        f = open('errores.txt', 'w')
        # estructura del mensaje de error: fecha + hora; tiempo de actividad; nombre del dispositivo; ip; error; memoria libre
    tserror = utime.time()
    msg_error = str(error_code) + str(tserror) + '\n'
    print('Mensaje de error: ', msg_error)
#    f.write('\n')
    f.write(msg_error)
    f.close()
    utime.sleep(5)


def check_errorfile():
    try:
        f = open('errores.txt', "r")
    except OSError:
        return
    rows = f.readline()
    rows = b'%s' % rows
    while rows:
        try:
            client.publish(topic_pub_error, rows)
            rows = f.readline()
            rows = b'%s' % rows
            utime.sleep(2)
        except OSError as e:
            print('No pudo leer')
            # Not working, lets boot
            # restart_and_reconnect()
    #  File read and reported, close and delete it
    f.close()
    os.remove('errores.txt')


def restart_and_reconnect():
    print('Failed to connect to MQTT broker. Reconnecting...')
    report_error(error_code=11)
    machine.reset()


def main_error():
    error = 'Reset por error en el Main ' + "{:02d}/{:02d}/{}".format(rtc.now()[2], rtc.now(
    )[1], rtc.now()[0]) + " {:02d}:{:02d}:{:02d}".format(rtc.now()[3], rtc.now()[4], rtc.now()[5])
    print(error)
    report_error(error_code=13)
    machine.reset()


def check_wlan():
    global rssi
    if not wlan.isconnected():
        print('Se perdio Wifi, se resetea')
        contwifi = 1
        print('Conectando WiFi...', wifi_ssid)
        wlan.connect(ssid=wifi_ssid, auth=(WLAN.WPA2, wifi_pass))
        while not wlan.isconnected():
            print('Inetento de coneccion WiFi:', contwifi)
            contwifi = contwifi + 1
            utime.sleep(2)
            if contwifi == resetwifi:
                print('No hay Wifi, se resetea!!!!!')
                report_error(error_code=10)
                utime.sleep(2)
                machine.reset()
            pass
    else:
        tuplawlan = wlan.joined_ap_info()
        rssi = str(tuplawlan[3])
        client.publish(topic_pub_rssi, rssi)
        print("Conexion WiFi up!!! Nivel de senal:", rssi)


def check_pybytes():
    global pybytes_count
    pybytes.send_ping_message()
    pybytes.send_signal(3, 'Keep Alive')
    if not pybytes.isconnected():
        print('Se perdio Pybytes, se rreconecta')
        pybytes_count = 1
        print('Conectando Pybytes...')
        pybytes.reconnect()
        while not pybytes.isconnected():
            print('Inetento de coneccion Pybytes:', pybytes_count)
            pybytes_count = pybytes_count + 1
            utime.sleep(2)
            if pybytes_count == resetwifi:
                print('No hay conexion con Pybytes, se resetea!!!!!')
                report_error(error_code=10)
                utime.sleep(2)
                machine.reset()
            pass
    else:
        print("Conexion Pybytes up!!!")

def check_internet():
    global rcv_int_fail, rcv_int_ok
    internet_up = ping('google.com')
    print("Respuesta de paquetes {0}".format(internet_up[1]))
    rcv_int_ok = int(internet_up[1])
#    s.send(bytes([0x30]) + bytes([0x03]) + bytes([0x00, 0x01]))
#    print('Paquete enviado por LoRa')
    if rcv_int_ok == 0:
        rcv_int_fail = rcv_int_fail + 1
        if rcv_int_fail == 3:
            rcv_int_fail = 0
            print('No hay internet')


def data_pub():
    global last_message, message_interval, rssi, u_min, u_max
    p = 0
    print('Empiezo lectura de Modbus')
    try:
        cheq_u1 = modbus_obj.read_holding_registers2(
            SLAVE_ADDR, REG_MODBUS[0], REG_SIZE[0], SIGNED)
        cheq_u1 = int('%i' % (float('%f' % (cheq_u1))))
        cheq_u2 = modbus_obj.read_holding_registers2(
            SLAVE_ADDR, REG_MODBUS[1], REG_SIZE[1], SIGNED)
        cheq_u2 = int('%i' % (float('%f' % (cheq_u2))))
        cheq_u3 = modbus_obj.read_holding_registers2(
            SLAVE_ADDR, REG_MODBUS[2], REG_SIZE[2], SIGNED)
        cheq_u3 = int('%i' % (float('%f' % (cheq_u3))))
        print(cheq_u1, cheq_u2, cheq_u3)
        if cheq_u1 < u_min or cheq_u1 > u_max:
            client.publish(TOPIC_VALUE[0], cheq_u1)
        if cheq_u2 < u_min or cheq_u2 > u_max:
            client.publish(TOPIC_VALUE[1], cheq_u2)
        if cheq_u3 < u_min or cheq_u3 > u_max:
            client.publish(TOPIC_VALUE[2], cheq_u3)
    except OSError as e:
        print('No se pudo leer Modbus')
    if (utime.time() - last_message) > message_interval:
        for count in range(0, 6):
            try:
                REG_VALUE[p] = modbus_obj.read_holding_registers2(
                    SLAVE_ADDR, REG_MODBUS[p], REG_SIZE[p], SIGNED)
                REG_VALUE[p] = str(int(('%i' % (float(('%f' % (REG_VALUE[p])))*1))))
                print("Registro de Modbus: ", REG_MODBUS[p])
                print("Valor de Registro: ", REG_VALUE[p])
                print('Topic para enviar:', TOPIC_VALUE[p])
                client.publish(TOPIC_VALUE[p], REG_VALUE[p])
                print('Mensaje enviado: ', p)
                p = p+1
                utime.sleep(2)
                last_message = utime.time()
#               print_publish()
            except OSError as e:
                print('No se pudo leer Modbus')

### Manejo de Alarmas

def check_alarm():
    global alm01_active, alm01_send, alm02_active, alm02_send, alm03_active, alm03_send, alm04_active, alm04_send, alm05_active, alm05_send
    print('Estado de Alarma 01 (P11) =', alm01())
    print('Estado de Alarma 02 (P12) =', alm02())
    print('Estado de Alarma 03 (P13) =', alm03())
    print('Estado de Alarma 04 (P14) =', alm04())
    print('Estado de Alarma 05 (P16) =', alm05())
    alm01_status = alm01()
    alm02_status = alm02()
    alm03_status = alm03()
    alm04_status = alm04()
    alm05_status = alm05()
    if (alm01_active == 1) and (alm01_send <= 2):
        client.publish(topic_pub_alm01, str(alm01_active))
        print('Se envio alarma 01 Activa = ', alm01_active)
        alm01_send = alm01_send + 1
        print('Contador de envios = ', alm01_send)
    if (alm01_status == 1) and (alm01_active == 1):
        alm01_active = 0
        alm01_send = 0
        client.publish(topic_pub_alm01, str(alm01_active))
### Alamrma 2
    if (alm02_status == 0) and (alm02_send <= 2):
        alm02_active = 1
        client.publish(topic_pub_alm02, str(alm02_active))
        print('Se envio alarma 02 Activa = ', alm02_active)
        alm02_send = alm02_send + 1
        print('Contador de envios = ', alm02_send)
    if (alm02_status == 1) and (alm02_active == 1):
        alm02_active = 0
        alm02_send = 0
        client.publish(topic_pub_alm02, str(alm02_active))
### Alamrma 3
    if (alm03_status == 0) and (alm03_send <= 2):
        alm03_active = 1
        client.publish(topic_pub_alm03, str(alm03_active))
        print('Se envio alarma 03 Activa = ', alm03_active)
        alm03_send = alm03_send + 1
        print('Contador de envios = ', alm03_send)
    if (alm03_status == 1) and (alm03_active == 1):
        alm03_active = 0
        alm03_send = 0
        client.publish(topic_pub_alm03, str(alm03_active))
### Alamrma 4
    if (alm04_status == 0) and (alm04_send <= 2):
        alm04_active = 1
        client.publish(topic_pub_alm04, str(alm04_active))
        print('Se envio alarma 04 Activa = ', alm04_active)
        alm04_send = alm04_send + 1
        print('Contador de envios = ', alm04_send)
    if (alm04_status == 1) and (alm04_active == 1):
        alm04_active = 0
        alm04_send = 0
        client.publish(topic_pub_alm04, str(alm04_active))
### Alamrma 5
    if (alm05_status == 0) and (alm05_send <= 2):
        alm05_active = 1
        client.publish(topic_pub_alm05, str(alm05_active))
        print('Se envio alarma 05 Activa = ', alm05_active)
        alm05_send = alm05_send + 1
        print('Contador de envios = ', alm05_send)
    if (alm05_status == 1) and (alm05_active == 1):
        alm05_active = 0
        alm05_send = 0
        client.publish(topic_pub_alm05, str(alm05_active))

def check_dht():
    readdht = dht1.read()
    if readdht.is_valid():
        print("Temperature: %d C" % readdht.temperature)
        print("Humidity: %d %%" % readdht.humidity)
        client.publish(topic_pub_temp, str(readdht.temperature))
        client.publish(topic_pub_hum, str(readdht.humidity))
    else:
        print('No se pudo leer sensor DHT')



#####  Inicializo la placa y los parametros de medicion
# Start WiFi Connection
wlan = WLAN(mode=WLAN.STA)

if not wlan.isconnected():
    contwifi = 1
    print('Conectando WiFi...', wifi_ssid)
    wlan.connect(ssid=wifi_ssid, auth=(WLAN.WPA2, wifi_pass))
    wlan.ifconfig()
    while not wlan.isconnected():
        print('Inetento de coneccion WiFi:', contwifi)
        contwifi = contwifi + 1
        utime.sleep(2)
        if contwifi == resetwifi:
            print('No hay Wifi, se resetea!!!!!')
            report_error(error_code=10)
            utime.sleep(2)
            machine.reset()
        pass
# Start MQTT Ubidots Connection
try:
    client = connect_mqtt()
except OSError as e:
    restart_and_reconnect()

# Send error file
check_errorfile()

### Alarmas
print('Configuro las alarmas /n/n/n')
alm01 = Pin('P11', mode=Pin.IN)
alm02 = Pin('P12', mode=Pin.IN)
alm03 = Pin('P13', mode=Pin.IN)
alm04 = Pin('P14', mode=Pin.IN)
alm05 = Pin('P16', mode=Pin.IN)

alm01_active = str(1)
alm01_send = 0
alm02_active = str(0)
alm02_send = 0
alm03_active = str(0)
alm03_send = 0
alm04_active = str(0)
alm04_send = 0
alm05_active = str(0)
alm05_send = 0

def alm01_handler_active(arg):
    global alm01_active
    print('Se disparo ALM01/n')
    alm01_active = 1
    print(alm01_active)

alm01.callback(Pin.IRQ_FALLING, alm01_handler_active, arg='ALM01')

client.publish(topic_pub_alm01, alm01_active)
client.publish(topic_pub_alm02, alm02_active)
client.publish(topic_pub_alm03, alm03_active)
client.publish(topic_pub_alm04, alm04_active)
client.publish(topic_pub_alm05, alm05_active)

# Start the loop
while True:
    try:
        lectura()
        check_wlan()
        check_pybytes()
        check_internet()
        check_alarm()
        check_dht()
        data_pub()
        pycom.heartbeat(False)
        pycom.rgbled(0x007f00) # green
        utime.sleep(3)
        pycom.rgbled(0x000000) # Off
        pycom.heartbeat(True)
#        utime.sleep(tsgral)
    except OSError as e:
        main_error()
