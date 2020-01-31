#include "point_cloud_player_window.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <QFileDialog>
#include <QMessageBox>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pcl/io/pcd_io.h>
#include "vtkRenderWindow.h"
#include "ui_point_cloud_player_window.h"
#include "driver.h"

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
  cloudLoadingIndex = 0;
  init();
}

PointCloudPlayer::~PointCloudPlayer() {
  delete ui;
}

void PointCloudPlayer::init() {
  viewer->setSize(cloudViewer->width(), cloudViewer->height());
  cloudViewer->SetRenderWindow(viewer->getRenderWindow());

  createPlaySpeedComboBox();

  connect (playSlider, SIGNAL (sliderMoved (const int &)), this, SLOT (playSliderMoved (const int &)));
  // connect (playSlider, SIGNAL (sliderPressed (const int &)), this, SLOT (playSliderPressed (const int &)));
  // connect (playSlider, SIGNAL (sliderReleased (const int &)), this, SLOT (playSliderReleased (const int &)));

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

void PointCloudPlayer::playSliderMoved(const int &value) {
  currentCloudIndex = value;
  updateDisplay(value);
}

void PointCloudPlayer::saveButtonPressed() {
  QString defaultFileName = QString("/%1").arg(clouds[currentCloudIndex].header.stamp);
  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Save Points"), defaultFileName, tr("Files (*pcd)"));

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
  frameNb->display(index + 1);
  playSlider->setValue(index);
  viewer->updatePointCloud<pcl::PointXYZI>(clouds[index].makeShared(), "cloud");
  viewer->spinOnce(0.0000000000001);
}

void PointCloudPlayer::addLidarAction(const CLidarConfig &config) {
  if (!lidar) {
    lidar->Stop();
    delete lidar;
    std::map<int, PointCloudT> empty;
    clouds.clear();
    clouds.swap(empty);
    cloudLoadingIndex = 0;
  }

  std::string ip = config.ip;
  std::string groupIp = "239.0.0.1";
  int port = config.port;
  std::string model = config.model;
  int returnType = config.returnType;
  float cutAngle = -1.0;
  int direction = -1;
  int version = -1;
  std::vector<std::string> correctionFileList;

  buttonsEnabled(false);

  lidar = new itd_lidar::lidar_driver::Driver(
    ip, groupIp, port, model,
    returnType, direction, version,
    correctionFileList, cutAngle,
    boost::bind(&PointCloudPlayer::lidarCallback, this, _1, _2),
    config.pcapFilePath);
  lidar->Start();
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

void PointCloudPlayer::lidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp) {
  clouds[++cloudLoadingIndex] = *cloud;
  if (timestamp == -10) {
    cloudLoadingIndex--;
    playSlider->setMaximum(cloudLoadingIndex);
    playSlider->setMinimum(1);
    currentCloudIndex = 0;
    cloudsSize = cloudLoadingIndex;
    buttonsEnabled(true);
    viewer->removeAllPointClouds();
    viewer->addPointCloud<pcl::PointXYZI>(clouds[currentCloudIndex].makeShared(), "cloud");
    updateDisplay(currentCloudIndex);
  }
}
