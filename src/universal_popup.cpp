#include "universal_popup.h"
#include "ui_universal_popup.h"

using namespace std::chrono_literals;

UniversalPopup::UniversalPopup(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::UniversalPopup)
{
  ui->setupUi(this);
  this->setAttribute(Qt::WA_DeleteOnClose);

  loadingIcon = new QMovie("/run/user/1000/doc/16eb4fe1/circle.gif");
  loadingIcon->setScaledSize(QSize(80,80));
  connect(loadingIcon,SIGNAL(finished()),this,SLOT(movieFinished()));
  loadingIcon->start();
}

UniversalPopup::~UniversalPopup()
{
  delete ui;
  delete loadingIcon;

  if(worker != nullptr)
    delete worker;
}

void UniversalPopup::setIcon(QIcon *icon){
  if(icon == nullptr){
    ui->icon->setMovie(loadingIcon);
    return;
  }

  currentIcon = icon;
  ui->icon->setPixmap(icon->pixmap(QSize(200,150)));
}

void UniversalPopup::setMainText(std::string &&text){
  ui->text->setText(QString::fromStdString(text));
}

void UniversalPopup::setMainText(std::string &text)
{
  ui->text->setText(QString::fromStdString(text));
}

void UniversalPopup::setPositiveButtonText(std::string &&buttonText){
  ui->positiveButton->setText(QString::fromStdString(buttonText));
}

void UniversalPopup::setPositiveButtonVisibility(bool visible)
{
  ui->positiveButton->setVisible(visible);
  ui->positiveButton->setEnabled(visible);
}

void UniversalPopup::setPositiveButtonCallback(std::function<bool ()> callable){
  this->callable = callable;
  worker = new WorkerThread(this, callable);
  connect(worker,SIGNAL(finished(bool)),this,SLOT(workerFinished(bool)));
}

void UniversalPopup::setNegativeButtonVisibility(bool visible)
{
  ui->pushButton->setVisible(visible);
  ui->pushButton->setEnabled(visible);
}

std::string UniversalPopup::getMainText()
{
  return ui->text->text().toStdString();
}

void UniversalPopup::movieFinished()
{
  loadingIcon->start();
}

void UniversalPopup::workerFinished(bool returnStatus)
{
  if(!returnStatus){

    ui->pushButton->setEnabled(true);
    ui->pushButton->setEnabled(true);

    ui->icon->setMovie(nullptr);
    ui->icon->setPixmap(currentIcon->pixmap(QSize(200,150)));

    ui->text->setText(QString::fromStdString(std::string("Something went wrong, try again.")));
    return;
  }

  close();
}


void UniversalPopup::on_pushButton_clicked()
{
  this->close();
}


void UniversalPopup::on_positiveButton_clicked()
{
  if(callable != nullptr){
    ui->text->setText(QString::fromStdString(std::string("Loading, please do not close the window")));

    ui->icon->setPixmap(QPixmap());
    ui->icon->setMovie(loadingIcon);

    ui->pushButton->setEnabled(false);
    ui->positiveButton->setEnabled(false);

    worker->start();
  }
}

void UniversalPopup::reject()
{
  if(worker != nullptr){
    if(!worker->isWorking())
      QDialog::reject();
  }
}

