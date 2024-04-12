from queue import LifoQueue

def enum_stk_and_recover(stk: LifoQueue):
    tmp = LifoQueue()
    try:
        while not stk.empty():
            item = stk.get()
            tmp.put(item)
            yield item
        return
    finally:
        while not tmp.empty():
            stk.put(tmp.get())
            

def enum_stk_and_empty(stk: LifoQueue):
    try:
        while not stk.empty():
            item = stk.get()
            yield item
        return
    finally:
        while not stk.empty():
            stk.get()

def enum_stk(stk: LifoQueue):
    '''不断枚举直达 stk为空或被外部中断，元素皆不会恢复'''
    while not stk.empty():
        item = stk.get()
        yield item
    return