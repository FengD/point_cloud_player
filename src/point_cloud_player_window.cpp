#include "point_cloud_player_window.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <QFileDialog>
#include <QMessageBox>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include "vtkRenderWindow.h"
#include "ui_point_cloud_player_window.h"

PointCloudPlayer::PointCloudPlayer(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::PointCloudPlayer) {
  ui->setupUi(this);

  cloudViewer = findChild<QVTKWidget*>("cloudView");
  viewer = new CloudVisualization();

  menuBar = findChild<QMenuBar*>("actionMenuBar");
  menuFirst = findChild<QMenu*>("menuFirst");
  actionAdd = findChild<QAction*>("add");

  playSlider = findChild<QSlider*>("playSlider");

  frameNb = findChild<QLCDNumber*>("frameNb");

  saveBtn = findChild<QPushButton*>("saveBtn");
  playBtn = findChild<QPushButton*>("playBtn");
  pauseBtn = findChild<QPushButton*>("pauseBtn");
  nextBtn = findChild<QPushButton*>("nextBtn");
  previousBtn = findChild<QPushButton*>("previousBtn");
  repeatBtn = findChild<QPushButton*>("repeatBtn");

  playSpeedComboBox = findChild<QComboBox*>("playSpeedComboBox");
  currentCloudIndex = -1;
  cloudsSize = -1;
  init();
}

PointCloudPlayer::~PointCloudPlayer() {
  delete ui;
}

void PointCloudPlayer::init() {
  cloudViewer->SetRenderWindow(viewer->getRenderWindow());
  createPlaySpeedComboBox();

  connect (playSlider, SIGNAL (valueChanged (const int &)), this, SLOT (playSliderValueChanged (const int &)));

  connect(menuBar, SIGNAL(triggered(QAction*)),
          this, SLOT(trigerMenu(QAction*)));
  connect(playSpeedComboBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(currentIndexChanged(const QString &)));

  connect (saveBtn,  SIGNAL (clicked ()), this, SLOT (saveButtonPressed ()));
  connect (playBtn,  SIGNAL (clicked ()), this, SLOT (playButtonPressed ()));
  connect (pauseBtn,  SIGNAL (clicked ()), this, SLOT (pauseButtonPressed ()));
  connect (nextBtn,  SIGNAL (clicked ()), this, SLOT (nextButtonPressed ()));
  connect (previousBtn,  SIGNAL (clicked ()), this, SLOT (previousButtonPressed ()));
  connect (repeatBtn,  SIGNAL (clicked ()), this, SLOT (repeatButtonPressed ()));
  buttonsEnabled(false);
}

void PointCloudPlayer::createPlaySpeedComboBox() {
  QStringList QList;
  QList.clear();
  QList << tr("x0.1")
        << tr("x0.5")
        << tr("x1.0")
        << tr("x1.5")
        << tr("x2.0")
        << tr("x3.0");
  playSpeedComboBox->clear();
  playSpeedComboBox->addItems(QList);
  playSpeedComboBox->setCurrentIndex(2);
}

void PointCloudPlayer::trigerMenu(QAction* act) {
  if(act->text() == "Add") {
    AddDialog addDialog;
    int ret = addDialog.exec();
    if (ret == AddDialog::Rejected)
      return;
    addLidarAction(addDialog.getLidarConfig());
  }
}

void PointCloudPlayer::playSliderValueChanged(const int &value) {

}

void PointCloudPlayer::saveButtonPressed() {
  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Save Points"), "/", tr("Files (*pcd)"));

  if (fileName.isEmpty()) {
    return;
  }

  if (currentCloudIndex < 0) {
    QMessageBox::about(NULL, "Error", "Invalid Point Cloud!");
    return;
  }

  fileName += ".pcd";
  pcl::io::savePCDFileBinary (fileName.toLatin1().data(), clouds[currentCloudIndex]);
}

void PointCloudPlayer::playButtonPressed() {
  playBtn->setEnabled(false);
  pauseBtn->setEnabled(true);
  nextBtn->setEnabled(false);
  previousBtn->setEnabled(false);
}

void PointCloudPlayer::pauseButtonPressed() {
  playBtn->setEnabled(true);
  pauseBtn->setEnabled(false);
  nextBtn->setEnabled(true);
  previousBtn->setEnabled(true);
}

void PointCloudPlayer::nextButtonPressed() {
  if (currentCloudIndex < cloudsSize) {
    currentCloudIndex++;
    previousBtn->setEnabled(true);
  } else {
    nextBtn->setEnabled(false);
  }
  updateDisplay(currentCloudIndex);
}

void PointCloudPlayer::previousButtonPressed() {
  if (currentCloudIndex > 0) {
    currentCloudIndex--;
    nextBtn->setEnabled(true);
  } else {
    previousBtn->setEnabled(false);
  }
  updateDisplay(currentCloudIndex);
}

void PointCloudPlayer::repeatButtonPressed() {

}

void PointCloudPlayer::currentIndexChanged(const QString &text) {}

void PointCloudPlayer::updateDisplay(const int &index) {
  frameNb->display(index);
  playSlider->setValue(index);
}

void PointCloudPlayer::addLidarAction(const CLidarConfig &config) {
  std::cout << config << std::endl;
  if (config.mode == 2) {
    buttonsEnabled(true);
  } else if (config.mode == 1) {
    buttonsEnabled(false);
  }
}

void PointCloudPlayer::buttonsEnabled(const bool &isEnable) {
  saveBtn->setEnabled(isEnable);
  playBtn->setEnabled(isEnable);
  pauseBtn->setEnabled(isEnable);
  nextBtn->setEnabled(isEnable);
  previousBtn->setEnabled(isEnable);
  repeatBtn->setEnabled(isEnable);
  playSlider->setEnabled(isEnable);
  frameNb->setEnabled(isEnable);
  playSpeedComboBox->setEnabled(isEnable);
}

// void MainWindow::lidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, double timestamp, std::string deviceName) {
//   (void) timestamp;
//   pcl::copyPointCloud(*cloud, lidars.find(deviceName)->second.cloud);
//   for (size_t i = 0; i < lidars.find(deviceName)->second.cloud.size (); i++) {
//     lidars.find(deviceName)->second.cloud.points[i].r = lidars.find(deviceName)->second.red;
//     lidars.find(deviceName)->second.cloud.points[i].g = lidars.find(deviceName)->second.green;
//     lidars.find(deviceName)->second.cloud.points[i].b = lidars.find(deviceName)->second.blue;
//   }
//   cloudDisplayUpdate();
// }
