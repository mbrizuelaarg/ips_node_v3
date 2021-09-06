import time
from . import simple
import machine
import os
from machine import RTC

class MQTTClient(simple.MQTTClient):

    DELAY = 2
    DEBUG = False

    def delay(self, i):
        utime.sleep(self.DELAY)

    def log(self, in_reconnect, e):
        if self.DEBUG:
            if in_reconnect:
                print("mqtt reconnect: %r" % e)
            else:
                print("mqtt: %r" % e)

    def reconnect(self):
        i = 0
        while 1:
            try:
                return super().connect(False)
                print('Reconectando')
            except OSError as e:
                print('No se esta reconectando', i)
                self.log(True, e)
                i += 1
                self.delay(i)
                if i == 15:
                    print('Se resetea por falla de reconnect')
                    try:
                        f = open ('errores.txt', "a")
                    except OSError:
                        f = open ('errores.txt', 'w')
                    ts = "{}, {:02d}, {:02d}".format(rtc.now()[0], rtc.now()[1], rtc.now()[2]) + ", {:02d}, {:02d}, {:02d}, 0, 0".format(rtc.now()[3], rtc.now()[4], rtc.now()[5])
                    ts = eval(ts)
                    ts = time.mktime(ts)
                    ts = ts + 946684800
                    msg_error = str(17) + str(ts) + '\n'
                    print('Mensaje de error: ', msg_error)
                    f.write(msg_error)
                    f.close()
                    time.sleep(5)
                    machine.reset()

    def publish(self, topic, msg, retain=False, qos=0):
        while 1:
            try:
                return super().publish(topic, msg, retain, qos)
            except OSError as e:
                self.log(False, e)
            self.reconnect()

    def wait_msg(self):
        while 1:
            try:
                print('Me voy al wait_msg de simple.py')
                return super().wait_msg()
            except OSError as e:
                self.log(False, e)
            self.reconnect()
