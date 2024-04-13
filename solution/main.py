import sys
import random

from core import Env
from scheduler import Scheduler
from log import main_logger

if __name__ == "__main__":

    # env负责管理各种全局变量，同时负责处理判题器的输入
    env = Env()

  
    # 最初输入的处理
    env.init_env()
    scheduler = Scheduler(env)
    print("OK")
    sys.stdout.flush()
    
    

    while True:
        try:
            # 每一帧的输入处理
            env.input()

            scheduler.run()
            
            print("OK")
            sys.stdout.flush()

        except EOFError:
            break

