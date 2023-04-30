//
// Created by wyb on 17-6-27.
// Modified (add template) by Weifan Gao on 23-04-30
//

#ifndef UNLOCKQUEUE_UNLOCK_QUEUE_H
#define UNLOCKQUEUE_UNLOCK_QUEUE_H

#include <stdint.h>
#include <atomic>
#include <algorithm>

static inline bool is_power_of_2(uint32_t num) {
  return (num != 0 && (num & (num-1)) == 0);
}

static inline uint32_t hightest_one_bit(uint32_t num) {
  num |= (num >> 1);
  num |= (num >> 2);
  num |= (num >> 4);
  num |= (num >> 8);
  num |= (num >> 16);
  return num - (num >> 1);
}

static inline uint32_t roundup_pow_of_two(uint32_t num) {
  return num > 1 ? hightest_one_bit((num - 1) << 1) : 1;
}

template <class T>
class UnlockQueue {
public:
  UnlockQueue(uint32_t size) : _in(0), _out(0) {
    _size = roundup_pow_of_two(size);
    _buffer = new T[_size];
  }

  ~UnlockQueue() {
    if (!_buffer)
      delete[] _buffer;
  }

  uint32_t Put(const T *buffer, uint32_t len) {
    if(len>_size - (_in - _out.load(std::memory_order_acq_rel)))//allow override
      _out.fetch_add(len, std::memory_order_acq_rel);
    // len = std::min(len, _size - (_in - _out.load(std::memory_order_acq_rel)));//not allow
    // len = std::min(len, _size - (_in - _out));
    // 通用内存屏障，保证out读到正确的值，可能另外一个线程在修改out
    // 用smp_mb是因为上面读out，下面写_buffer
    // smp_mb();

    uint32_t l = std::min(len, _size - (_in & (_size - 1)));
    memcpy(_buffer + (_in & (_size - 1)), buffer, l*sizeof(T));
    memcpy(_buffer, buffer + l, (len - l)*sizeof(T));
    // 写内存屏障，保证先写完_buffer，再修改in
    // smp_wmb();
    _in.fetch_add(len, std::memory_order_release);
    //_in += len;

    return len;
  }

  uint32_t Get(T *buffer, uint32_t len) {
    len = std::min(len, _in.load(std::memory_order_acquire) - _out);
    // 读内存屏障，保证读到正确的in，可能另外一个线程正在修改in
    // 用smp_rmb是因为上面读in，下面读_buffer
    //smp_rmb();

    uint32_t l = std::min(len, _size - (_out & (_size - 1)));
    memcpy(buffer, _buffer + (_out & (_size - 1)), l*sizeof(T));
    memcpy(buffer + l, _buffer, (len - l)*sizeof(T));
    // 通用内存屏障，保证先把buffer读出去，再修改out
    // 上面读_bufer，下面写out
    // smp_mb();
    _out.fetch_add(len, std::memory_order_acq_rel);
    //_out += len;

    return len;
  }
  uint32_t Get_wait(T *buffer, uint32_t len, uint32_t timeout_ms) {
    uint32_t cnt = 0;
    while (length()<len){
      if(cnt++>=timeout_ms)
        break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return Get(buffer, len);
  }

  inline uint32_t size() { return _size; }
  inline uint32_t length() { return (_in - _out); }
  inline bool empty() { return _in <= _out; }

private:
  T *_buffer;
  uint32_t _size;
  std::atomic<uint32_t> _in;
  std::atomic<uint32_t> _out;
};



#endif //UNLOCKQUEUE_UNLOCK_QUEUE_H
