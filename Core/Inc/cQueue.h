/*
 * cQueue.h
 *
 *  Created on: Mar 15, 2023
 *      Author: IoT02
 */

#ifndef INC_CQUEUE_H_
#define INC_CQUEUE_H_

#include "icm209482.h"

#define CQUEUE_SIZE 20

typedef axisesAll element;

typedef struct{
  element cQueue[CQUEUE_SIZE];
  uint8_t front, rear;
}QueueType;

QueueType* createCQueue();
int isCQueueEmpty(QueueType* cQ);
int isCQueueFull(QueueType* cQ);
void enCQueue(QueueType* cQ,element item );
element deCQueue(QueueType* cQ);
element peekCQ(QueueType* cQ);
int sizeOfCQueue(QueueType* cQ);

#endif /* INC_CQUEUE_H_ */
