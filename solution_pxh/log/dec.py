from typing import Callable
from time import time

from .logger import main_logger

def func_timer(func: Callable):
    def func_wrapper(*args, **kwargs):
        t0_114514 = time()
        result = func(*args, **kwargs)
        t1_114514 = time()
        main_logger.info(f"{func.__name__} cost time: {(t1_114514 - t0_114514)*1000} ms")
        return result

    return func_wrapper