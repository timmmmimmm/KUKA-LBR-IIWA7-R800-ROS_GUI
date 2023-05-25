#include "worker_thread.h"

WorkerThread::WorkerThread(QObject *parent) : QThread{parent}{}

WorkerThread::WorkerThread(QObject *parent, std::function<bool ()> callable)
  : WorkerThread(parent)
{
  this->callable = callable;
}

void WorkerThread::run(){

  if(callable != nullptr){
    working = true;
    if(!callable()){
      working = false;
      emit finished(false);
      return;
    }
  }

  working = false;
  emit finished(true);
}

bool WorkerThread::isWorking()
{
  return working;
}
