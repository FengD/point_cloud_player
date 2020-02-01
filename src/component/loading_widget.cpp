#include "loading_widget.h"
#include "ui_loading_widget.h"

LoadingWidget::LoadingWidget(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::LoadingWidget) {
  ui->setupUi(this);

  setWindowFlags(Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);

  int frmX = width();
  int frmY = height();

  QDesktopWidget w;
  int deskWidth = w.width();
  int deskHeight = w.height();

  QPoint movePoint(deskWidth / 2 - frmX / 2, deskHeight / 2 - frmY / 2);
  move(movePoint);

  QMovie *movie = new QMovie(":/home/ding/Documents/point_cloud_player/plugins/loading.gif");
  ui->loadingLabel->setMovie(movie);
  movie->start();
}

LoadingWidget::~LoadingWidget() {
  delete ui;
}
