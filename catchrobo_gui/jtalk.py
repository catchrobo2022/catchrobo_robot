# -*- coding: utf-8 -*-
import subprocess
from datetime import datetime
import os

def jtalk(t):
    PATH = os.path.dirname(__file__)
    open_jtalk=['open_jtalk']
    mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
    htsvoice=['-m','/usr/share/hts-voice/mei/mei_normal.htsvoice']
    speed=['-r','0.7']
    outwav=['-ow', PATH +'/voice/jtalk_gui2.wav']
    cmd=open_jtalk+mech+htsvoice+speed+outwav
    c = subprocess.Popen(cmd,stdin=subprocess.PIPE)
    c.stdin.write(t.encode())
    c.stdin.close()
    c.wait()
    aplay = ['aplay','-q',PATH +'/voice/jtalk.wav']
    wr = subprocess.Popen(aplay)

def gtalk(t):
    PATH = os.path.dirname(__file__)
    mpg123 = ['mpg123','-q',PATH+'/voice/'+t]
    wr = subprocess.Popen(mpg123)

def say_datetime():
    d = datetime.now()
    text = '%s月%s日、%s時%s分%s秒' % (d.month, d.day, d.hour, d.minute, d.second)
    jtalk(text)

if __name__ == '__main__':
    say_datetime()
