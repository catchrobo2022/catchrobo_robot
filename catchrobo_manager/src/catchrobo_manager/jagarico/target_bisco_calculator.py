#!/usr/bin/env python
# -*- coding: utf-8 -*-


import numpy as np
import copy


class TargetBiscoCalculator:
    def __init__(self):
        self._can_go_common = False
        
    def setCanGoCommon(self, flag):
        self._can_go_common = flag
    # [TODO]
    def calcTargetTwin(self, database):
        biscos = copy.deepcopy(database)

        # our area
        in1 = in2 = in3 = in4 = in5 = in6 = 0
        count = 0
        for i in range(6):
            # represents the state of biscos next to each other.
            val = 2*biscos.isExist(26-i-6)+biscos.isExist(26-i)

            if val == 2:  # ox
                if i == 0:
                    biscos.updateState(26-i-12, "priority", 1)
                if i == 1:
                    biscos.updateState(26-i-12, "priority", 3)
                if i == 2:
                    biscos.updateState(26-i-12, "priority", 213)
                if i == 3:
                    biscos.updateState(26-i-12, "priority", 215)
                if i == 4:
                    biscos.updateState(26-i-12, "priority", 217)
                if i == 5:
                    biscos.updateState(26-i-12, "priority", 219)

            if val == 1:  # xo
                biscos.updateState(
                    26-i, "priority", biscos.getState(26-i, "priority")+226)

                if biscos.getState(26, "priority") == 227 and in1 == 0:
                    count = 1
                    in1 = 1
                    biscos.updateState(25, "priority", 1)
                    biscos.updateState(19, "priority", 2)
                    biscos.updateState(24, "priority", 3)
                    biscos.updateState(18, "priority", 4)
                    print("count=1\n")

                if (biscos.getState(25, "priority") == 227 or biscos.getState(25, "priority") == 229) and in2 == 0:
                    in2 = 1
                    if count == 0:  # have to get 2 more
                        count = 2
                        biscos.updateState(24, "priority", 3)
                        biscos.updateState(18, "priority", 4)
                        print("count=2\n")
                    elif count == 1:  # have to get all 4
                        count = 3
                        biscos.updateState(24, "priority", 1)
                        biscos.updateState(18, "priority", 2)
                        biscos.updateState(23, "priority", 3)
                        biscos.updateState(17, "priority", 4)
                        biscos.updateState(25, "priority", 228)
                        print("count=3\n")

                if (biscos.getState(24, "priority") == 229 or biscos.getState(24, "priority") == 227) and in3 == 0:
                    in3 = 1
                    if count == 1:
                        count = 4
                        biscos.updateState(25, "priority", 1)
                        biscos.updateState(19, "priority", 2)
                        biscos.updateState(23, "priority", 3)
                        biscos.updateState(17, "priority", 4)
                        print("count=4\n")

                    elif count == 2:
                        count = 5
                        biscos.updateState(23, "priority", 3)
                        biscos.updateState(17, "priority", 4)
                        biscos.updateState(25, "priority", 228)
                        print("count=5\n")

                    elif count == 3:
                        count = 6
                        biscos.updateState(23, "priority", 1)
                        biscos.updateState(17, "priority", 2)
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        biscos.updateState(24, "priority", 229)
                        print("count=6\n")

                if (biscos.getState(23, "priority") == 227 or biscos.getState(23, "priority") == 229 or biscos.getState(23, "priority") == 230) and in4 == 0:
                    in4 = 1
                    if count == 3:
                        count = 7
                        biscos.updateState(24, "priority", 1)
                        biscos.updateState(18, "priority", 2)
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        print("count=7\n")

                    elif count == 4:
                        count = 8
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        biscos.updateState(24, "priority", 228)
                        print("count=8\n")

                    elif count == 5:
                        count = 9
                        biscos.updateState(22, "priority", 3)
                        biscos.updateState(16, "priority", 4)
                        biscos.updateState(23, "priority", 230)
                        print("count=9\n")

                    elif count == 6:
                        count = 10
                        biscos.updateState(22, "priority", 1)
                        biscos.updateState(16, "priority", 2)
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(23, "priority", 230)
                        print("count=10\n")

                if (biscos.getState(22, "priority") == 227 or biscos.getState(22, "priority") == 229) and in5 == 0:
                    in5 = 1
                    if count == 6:
                        count = 11
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 230)
                        print("count=11\n")

                    elif count == 7:
                        count = 12
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 230)
                        print("count=12\n")

                    elif count == 8:
                        count = 13
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 230)
                        print("count=13\n")

                    elif count == 9:
                        count = 14
                        biscos.updateState(21, "priority", 3)
                        biscos.updateState(15, "priority", 4)
                        biscos.updateState(22, "priority", 231)
                        print("count=14\n")

                    elif count == 10:  # not twin is better
                        count = 15
                        biscos.updateState(21, "priority", 1)
                        biscos.updateState(15, "priority", 2)
                        biscos.updateState(26, "priority", 3)
                        biscos.updateState(14, "priority", 4)
                        biscos.updateState(22, "priority", 231)
                        print("count=15\n")

                if (biscos.getState(21, "priority") == 227 or biscos.getState(21, "priority") == 229 or biscos.getState(21, "priority") == 231) and in6 == 0:
                    in6 = 1
                    if count == 10:  # not twin is better
                        count = 16
                        biscos.updateState(26, "priority", 3)
                        biscos.updateState(14, "priority", 4)
                        biscos.updateState(21, "priority", 231)
                        print("count=16\n")

                    elif count == 11:  # not twin is better
                        count = 17
                        biscos.updateState(26, "priority", 3)
                        biscos.updateState(14, "priority", 4)
                        biscos.updateState(21, "priority", 231)
                        print("count=17\n")

                    elif count == 12:  # not twin is better
                        count = 18
                        biscos.updateState(26, "priority", 3)
                        biscos.updateState(14, "priority", 4)
                        biscos.updateState(21, "priority", 231)
                        print("count=18\n")

                    elif count == 13:  # not twin is better
                        count = 19
                        biscos.updateState(26, "priority", 3)
                        biscos.updateState(14, "priority", 4)
                        biscos.updateState(21, "priority", 231)
                        print("count=19\n")

                    elif count == 14:  # not twin is better
                        count = 20
                        biscos.updateState(25, "priority", 3)
                        biscos.updateState(13, "priority", 4)
                        biscos.updateState(21, "priority", 232)
                        print("count=20\n")

                    elif count == 15:  # not twin is better
                        count = 21
                        biscos.updateState(26, "priority", 1)
                        biscos.updateState(14, "priority", 2)
                        biscos.updateState(25, "priority", 3)
                        biscos.updateState(13, "priority", 4)
                        biscos.updateState(21, "priority", 232)
                        print("count=21\n")

        # common area

        for j in range(7):
            check = 2*biscos.isExist(7-j)+biscos.isExist(6-j)
            # print(check)
            if check == 2:
                biscos.updateState(
                    7-j, "priority", biscos.getState(7-j, "priority")+8)
        for m in range(7):
            check = 2*biscos.isExist(7-m)+biscos.isExist(6-m)
            # print(check)
            if check == 3:
                biscos.updateState(6-m, "priority", 106+m)

        '''
        # debag
        print("in1=",in1)
        print("in2=",in2)
        print("in3=",in3)
        print("in4=",in4)
        print("in5=",in5)
        print("in6=",in6)
        '''

        # priority check debag
        for h in range(6):
            print(biscos.getState(8-h, "priority"), biscos.getState(14-h, "priority"),
                  biscos.getState(20-h, "priority"), biscos.getState(26-h, "priority"))
        for l in range(3):
            print(biscos.getState(2-l, "priority"))
        print("\n")

        first = self.getMininumPriorityId(biscos)
        if first is None:
            return None, None

        if first >= 2 and first <= 7:
            check = 2*biscos.isExist(first)+biscos.isExist(first-1)
            print(2*biscos.isExist(first))
            if check == 2:
                print("abc")
                return first, None
        elif first == 1:
            check = 2*biscos.isExist(first)+biscos.isExist(first-1)
            if check == 2:
                return first, None
        elif first == 0:
            return first, None

        biscos.delete(first)
        second = self.getMininumPriorityId(biscos)

        return first, second

    def getMininumPriorityId(self, database):
        exist = database._objects["exist"]
        # print(self._objects[exist]["priority"])
        if np.sum(exist) == 0:
            minimum = None
        else:
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum

    def isNeighbor(self, biscos, first, second):
        ret = False
        if first is None or second is None:
            return False
        areas = [biscos.getState(first, "my_area"),
                 biscos.getState(second, "my_area")]
        if areas[0] == areas[1] == True:
            if abs(first - second) == 6:
                ret = True
        elif areas[0] == areas[1] == False:
            if abs(first - second) == 1:
                ret = True
        return ret


'''
num

08 14 20 26
07 13 19 25
06 12 18 24
05 11 17 23
04 10 16 22
03 09 15 21
02
01

priority count=0

999 221 002 001 
105 222 004 003
106 223 214 213
107 224 216 215
108 225 218 217
109 226 220 219
110 
111
112

priority count=1 

99 21 xx 27 
05 22 02 01
06 23 04 03
07 24 16 15
08 25 18 17
09 26 20 19
10 
11
12

priority count=2
03 04

99 21 02 01 
05 22 xx 29
06 23 04 03
07 24 16 15
08 25 18 17
09 26 20 19
10 
11
12

priority count=3

99 21 xx 27 
05 22 xx 27(=28)
06 23 02 01
07 24 04 03
08 25 18 17
09 26 20 19
10 
11
12


priority count=4
05 06

99 21 xx 27 
05 22 02 01
06 23 xx 29
07 24 04 03
08 25 18 17
09 26 20 19
10 
11
12


priority count=5
03 04 28

99 21 02 01 
05 22 xx 29(=28)
06 23 xx 29
07 24 04 03
08 25 18 17
09 26 20 19
10 
11
12


priority count=6
01 02 03 04

99 21 xx 27 
05 22 xx 28
06 23 xx 27(=29)
07 24 02 01
08 25 04 03
09 26 20 19
10 
11
12



priority count=7

99 21 xx 27 
05 22 xx 28
06 23 02 01
07 24 xx 29
08 25 04 03
09 26 20 19
10 
11
12


priority count=8

99 21 xx 27 
05 22 02 01
06 23 xx 29(=28)
07 24 xx 29
08 25 04 03
09 26 20 19
10 
11
12


priority count=9
03 04 30

99 21 02 01 
05 22 xx 28
06 23 xx 29
07 24 xx 29(=30)
08 25 04 03
09 26 20 19
10 
11
12


priority count=10
01 02 03 04 30

99 21 xx 27 
05 22 xx 28
06 23 xx 29
07 24 xx 27(=30)
08 25 02 01
09 26 04 03
10 
11
12


priority count=11
03 04 30

99 21 xx 27 
05 22 xx 28
06 23 xx 29
07 24 02 01
08 25 xx 29(=30)
09 26 04 03
10 
11
12


priority count=12
03 04 30

99 21 xx 27 
05 22 xx 28
06 23 02 01
07 24 xx 29
08 25 xx 29(=30)
09 26 04 03
10 
11
12


priority count=13
03 04 30

99 21 xx 27 
05 22 02 01
06 23 xx 28
07 24 xx 29
08 25 xx 29(=30)
09 26 04 03
10 
11
12


priority count=14
03 04 31

99 21 02 01 
05 22 xx 28
06 23 xx 29
07 24 xx 30
08 25 xx 29(=31)
09 26 04 03
10 
11
12


priority count=15
01 02 03 04 31

99 04 xx 03 
05 22 xx 28
06 23 xx 29
07 24 xx 30
08 25 xx 27(=31)
09 26 02 01
10 
11
12


priority count=16
01 02 03 04 31

99 04 xx 03 
05 22 xx 28
06 23 xx 29
07 24 xx 30
08 25 02 01
09 26 xx 29(=31)
10 
11
12


priority count=17

99 04 xx 03 
05 22 xx 28
06 23 xx 29
07 24 02 01
08 25 xx 30
09 26 xx 29(=31)
10 
11
12


priority count=18
07 37

99 04 xx 03 
05 22 xx 28
06 23 02 01
07 24 xx 29
08 25 xx 30
09 26 xx 29(=31)
10 
11
12


priority count=19
05 35

99 04 xx 03
05 22 02 01
06 23 xx 28
07 24 xx 29
08 25 xx 30
09 26 xx 29(=31)
10 
11
12


priority count=20
03 35

99 21 02 01 
05 04 xx 03
06 23 xx 29
07 24 xx 30
08 25 xx 31
09 26 xx 29(=32)
10 
11
12

priority count=21
01 02 03 37

99 02 xx 01 
05 04 xx 03
06 23 xx 29
07 24 xx 30
08 25 xx 31
09 26 xx 27(=32)
10 
11
12

'''
