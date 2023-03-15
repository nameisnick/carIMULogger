/*
 * cQueue.c
 *
 *  Created on: Mar 15, 2023
 *      Author: IoT02
 */

#include "cQueue.h"

QueueType* createCQueue(){
  QueueType* cQ;
  cQ = (QueueType*)malloc(sizeof(QueueType));
  cQ->front = 0;
  cQ->rear = 0;
}
int isCQueueEmpty(QueueType* cQ)
{
  if(cQ -> front == cQ -> rear){
    return 1;
  }else{
    return 0;
  }
}
int isCQueueFull(QueueType* cQ)
{
  if(((cQ->rear + 1)%CQUEUE_SIZE) == cQ -> front){
    return 1;
  }else{
    return 0;
  }
}
void enCQueue(QueueType* cQ,element item )
{
  cQ->rear = (cQ->rear + 1) % CQUEUE_SIZE;
  cQ->cQueue[cQ->rear] = item;
}
element deCQueue(QueueType* cQ)
{
  cQ->front = (cQ->front + 1) % CQUEUE_SIZE;
  return cQ->cQueue[cQ->front];
}
element peekCQ(QueueType* cQ);
//int sizeOfCQueue(QueueType* cQ)
//{
//
//
//  return cQ->
//}
