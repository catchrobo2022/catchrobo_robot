#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import copy

class TargetJagaricoCalculator:
    def __init__(self):
        self._can_go_common = False
        self._miss_count=0
        
    def setCanGoCommon(self, flag):
        self._can_go_common = flag

    # 1個目を取り損なったら、手動でお願いします。
    def calcTarget(self, database):
        jagarico = copy.deepcopy(database)   
        target=self.getMininumPriorityId(jagarico)

        return target

    def getMininumPriorityId(self, database):
        #"exist"の列の抽出
        exist = database._objects["exist"]

        if np.sum(exist) == 0:
            minimum = None
        else:
            # "exist"がtrueのみの"priority"の列 # "exist"がfalseだと出力されない
            minimum = database._objects[exist]["priority"].idxmin()
        return minimum





### error algorizm ###

## if use when need to think about when robot can pick the first jagarico
# 1個目のじゃがりこ取れない場合は下記の感じでやる
# この場合は、ミスった1個目の処理をどうするかとか結構話さないといけない（exsistをfalseにするのか）
# 正直、もうミスったら手動でよろしく
'''
###[BUG]この関数を使う度に、targetが変わる可能性あり（_can_go_commonがfalseだと）
def calcTarget(self, database):
    jagarico = copy.deepcopy(database)

    # 本当はこんな面倒なことをしなくていい
    # ただindex24からforで-1してpriority-200して、19以外っていうif文を入れるだけ

    # my_area内のpriorityの昇順に並び替え、indexの値を取得
    df_jagarico = pd.DataFrame(data=jagarico._objects)
    my_area_jagarico = df_jagarico.iloc[9:25]
     priority_sort = my_area_jagarico.sort_values("priority")
      index_num = priority_sort.index.values
       # print("index num",index_num)

       ###[TODO]_can_go_commonを利用する
       # _can_go_commonがfalseだとmy_area内で取りに行く
       if(self._can_go_common == False):
            if(self._miss_count != 0):
                for i in range(self._miss_count):
                    update_id = index_num[self._miss_count+i]
                    # print("update_id",update_id)
                    jagarico.updateState(
                        update_id, "priority", priority_sort.loc[update_id, "priority"]-200)
            self._miss_count += 1
            # my area内のを全部取り損なったとき
            if(self._miss_count == 15):
                print("GAME OVER")
                return None

        # debug用
        check = pd.DataFrame(data=jagarico._objects)
        check2 = check.loc[:, "priority"]
        print(check2)
        #

        target = self.getMininumPriorityId(jagarico)

        return target
'''
