import time
import os

# sudoPassword = 'mypass'
t = time.strftime('%Y-%m-%d-Time-%H-%M', time.localtime(time.time()))
command = 'jpnevulator -r -g -e 50000 -t /dev/ttyUSB1  | tee ' + '/home/pi/mm-Wave-Radar-IWR6843AOPEVM/raw_dump/data'+ t +'.txt'
# p = os.system('echo %s|sudo -S %s' % (sudoPassword, command))
p = os.system('sudo %s' % (command))



 