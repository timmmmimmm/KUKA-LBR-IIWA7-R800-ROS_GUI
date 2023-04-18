#pragma once

#include <QDialog>
#include <QIcon>
#include <QMovie>
#include <QCloseEvent>
#include <functional>

#include <ros/ros.h>
#include "worker_thread.h"

namespace Ui {
class UniversalPopup;
}

class UniversalPopup : public QDialog
{
  Q_OBJECT

public:
  explicit UniversalPopup(QWidget *parent = nullptr);
  ~UniversalPopup();

  void setIcon(QIcon *icon = nullptr);

  void setMainText(std::string&& );
  void setMainText(std::string& );

  void setPositiveButtonText(std::string&& );
  void setPositiveButtonVisibility(bool visible);
  void setPositiveButtonCallback(std::function<bool()>);

  void setNegativeButtonVisibility(bool visible);

  std::string getMainText();

public slots:
  void movieFinished();
  void workerFinished(bool returnStatus);

private slots:
  void on_pushButton_clicked();

  void on_positiveButton_clicked();

private:
  Ui::UniversalPopup *ui;
  QMovie *loadingIcon;
  WorkerThread *worker;
  QIcon *currentIcon;
  void reject();

  std::function<bool()> callable;
};

