//
// Created by wyb on 17-6-28.
// Modified (add template) by Weifan Gao on 23-04-30
//

#include "unlock_queue.h"
#include <iostream>
#include <cstdint>
#include <thread>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <cstdlib>

using namespace std;

void produce_func(UnlockQueue<uint8_t> *queue, uint32_t num) {
  for (uint32_t i = 0; i < num; ++i) {
    char buf[10];
    sprintf(buf, "%04u", i);
    queue->Put((uint8_t*)buf, 5);
    printf("%s -->\n", buf);fflush(stdout);
  }
}

void consume_func(UnlockQueue<uint8_t> *queue, uint32_t num) {
  char buf[10];
  for (uint32_t i = 0; i < num; ++i) {
    while (queue->empty())
      ;
    int ret = queue->Get((uint8_t*)buf, 5);
    printf("        --> %s\n", buf);
    fflush(stdout);
  }
}

int main() {
  UnlockQueue<uint8_t> *queue = new UnlockQueue<uint8_t>(20000);
  cout << queue->size() << endl;
  thread producer(produce_func, queue, 1000);
  thread consumer(consume_func, queue, 1000);
  consumer.join();
  producer.join();
  delete queue;
}