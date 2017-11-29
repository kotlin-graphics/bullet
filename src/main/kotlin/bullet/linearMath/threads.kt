package bullet.linearMath

/** SpinMutex -- lightweight spin-mutex implemented with atomic ops, never puts a thread to sleep because it is designed
 *  to be used with a task scheduler which has one thread per core and the threads don't sleep until they run out of
 *  tasks. Not good for general purpose use. */
class SpinMutex{
    var lock = 0

//    void lock();
//    void unlock();
//    bool tryLock();
}