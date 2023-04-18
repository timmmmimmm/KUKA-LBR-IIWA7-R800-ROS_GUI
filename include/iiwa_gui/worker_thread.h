#pragma once

#include <QObject>
#include <QThread>
#include <functional>

class WorkerThread : public QThread
{
  Q_OBJECT
public:
  explicit WorkerThread(QObject *parent = nullptr);
  explicit WorkerThread(QObject *parent, std::function<bool()> callable);

  void run();
  bool isWorking();

signals:
  void finished(bool returnState);

private:
  std::function<bool()> callable;
  bool working;
};

