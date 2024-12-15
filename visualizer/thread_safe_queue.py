import threading
from queue import Queue
from typing import Generic, TypeVar

T = TypeVar('T')

class ThreadSafeQueue(Queue, Generic[T]):
    def __init__(self, maxsize = 0):
        super().__init__(maxsize)
        self._lock = threading.RLock()
        
    def put(self, item: T, block = True, timeout = None):
        self._lock.acquire()
        ret = super().put(item, block, timeout)
        self._lock.release()
        
        return ret
    
    def put_nowait(self, item: T):
        self._lock.acquire()
        ret = super().put_nowait(item)
        self._lock.release()
        
        return ret
    
    def get(self, block = True, timeout = None) -> T:
        self._lock.acquire()
        ret = super().get(block, timeout)
        self._lock.release()
        
        return ret
    
    def get_nowait(self) -> T:
        self._lock.acquire()
        ret = super().get_nowait()
        self._lock.release()
        
        return ret