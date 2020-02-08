// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#include "point_cloud_player_window.h"
#include <math.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <QFileDialog>
#include <QMessageBox>
#include "vtkRenderWindow.h"
#include "ui_point_cloud_player_window.h"
#include "cloud_visualization.h"
#include "driver.h"

PointCloudPlayer::PointCloudPlayer(QWidget *parent) :
  QMainWindow(parent),
  ui_(new Ui::PointCloudPlayer),
  current_cloud_index_(-1),
  clouds_size_(-1),
  load_cloud_index_(0),
  play_speed_(-1.0) {
  ui_->setupUi(this);
  viewer_ = new itd_lidar::CloudVisualization();
  InitParam();
  InitSlotConnect();
}

PointCloudPlayer::~PointCloudPlayer() {
  delete ui_;
  delete viewer_;
}

void PointCloudPlayer::InitParam() {
  viewer_->setSize(ui_->cloudViewer->width(), ui_->cloudViewer->height());
  ui_->cloudViewer->SetRenderWindow(viewer_->getRenderWindow());
  CreatePlaySpeedComboBox();
  ButtonsEnabled(false);
}

void PointCloudPlayer::InitSlotConnect() {
  connect(ui_->playSlider, SIGNAL(sliderMoved(const int &)),
          this, SLOT(playSliderMoved(const int &)));
  connect(ui_->actionMenuBar, SIGNAL(triggered(QAction *)),
          this, SLOT(trigerMenu(QAction *)));
  connect(ui_->playSpeedComboBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(currentIndexChanged(const QString &)));
  connect(ui_->saveBtn, SIGNAL(clicked()),
          this, SLOT(saveButtonPressed()));
  connect(ui_->playBtn, SIGNAL(clicked()),
          this, SLOT(playButtonPressed()));
  connect(ui_->pauseBtn, SIGNAL(clicked()),
          this, SLOT(pauseButtonPressed()));
  connect(ui_->nextBtn, SIGNAL(clicked()),
          this, SLOT(nextButtonPressed()));
  connect(ui_->previousBtn, SIGNAL(clicked()),
          this, SLOT(previousButtonPressed()));
  connect(ui_->repeatBtn, SIGNAL(clicked()),
          this, SLOT(repeatButtonPressed()));
}

void PointCloudPlayer::CreatePlaySpeedComboBox() {
  QStringList QList;
  QList.clear();
  QList << tr("0.1") << tr("0.5") << tr("1.0")
        << tr("1.5") << tr("2.0") << tr("3.0");
  ui_->playSpeedComboBox->clear();
  ui_->playSpeedComboBox->addItems(QList);
  ui_->playSpeedComboBox->setCurrentIndex(2);
}

void PointCloudPlayer::trigerMenu(QAction* act) {
  if(act->text() == "Add") {
    AddDialog addDialog;
    int ret = addDialog.exec();
    if (ret == AddDialog::Rejected) {
      return;
    }
    CLidarConfig config = addDialog.GetLidarConfig();
    AddLidarAction(config);
  }
}

void PointCloudPlayer::playSliderMoved(const int &value) {
  current_cloud_index_ = value - 1;
  UpdateDisplay(current_cloud_index_);
}

void PointCloudPlayer::saveButtonPressed() {
  QString defaultFileName = QString("/%1").arg(clouds_[current_cloud_index_].header.stamp);
  QString fileName = QFileDialog::getSaveFileName(this,
    tr("Save Points"), defaultFileName, tr("Files (*pcd)"));

  if (fileName.isEmpty()) {
    return;
  }

  if (current_cloud_index_ < 0) {
    QMessageBox::about(NULL, "Error", "Invalid Point Cloud!");
    return;
  }

  fileName += ".pcd";
  pcl::io::savePCDFileBinary (fileName.toLatin1().data(), clouds_[current_cloud_index_]);
}

void PointCloudPlayer::playButtonPressed() {
  UpdatePushButtons(false);
}

void PointCloudPlayer::pauseButtonPressed() {
  UpdatePushButtons(true);
}

void PointCloudPlayer::UpdatePushButtons(const bool &isEnable) {
  // ui_->playBtn->setEnabled(isEnable);
  // ui_->pauseBtn->setEnabled(!isEnable);
  ui_->nextBtn->setEnabled(isEnable);
  ui_->previousBtn->setEnabled(isEnable);
  ui_->saveBtn->setEnabled(isEnable);
}

void PointCloudPlayer::nextButtonPressed() {
  if (current_cloud_index_ < clouds_size_) {
    current_cloud_index_++;
    ui_->previousBtn->setEnabled(true);
  } else {
    ui_->nextBtn->setEnabled(false);
  }
  UpdateDisplay(current_cloud_index_);
}

void PointCloudPlayer::previousButtonPressed() {
  if (current_cloud_index_ >= 0) {
    current_cloud_index_--;
    ui_->nextBtn->setEnabled(true);
  } else {
    ui_->previousBtn->setEnabled(false);
  }
  UpdateDisplay(current_cloud_index_);
}

void PointCloudPlayer::repeatButtonPressed() {}

void PointCloudPlayer::currentIndexChanged(const QString &text) {
  play_speed_ = text.toFloat();
}

void PointCloudPlayer::UpdateDisplay(const int &index) {
  ui_->frameNb->display(index + 1);
  ui_->playSlider->setValue(index + 1);
  viewer_->updatePointCloud<pcl::PointXYZI>(clouds_[index].makeShared(), "cloud");
  viewer_->spinOnce(0.0000000000001);
}

void PointCloudPlayer::AddLidarAction(const CLidarConfig &config) {
  if (config.mode == 2 && config.pcapFilePath == "") {
    QMessageBox::about(NULL, "Error", "No PCAP given!");
    return;
  }

  if (NULL == lidar_) {
    // lidar_->Stop();
    delete lidar_;
  }

  std::map<int, PointCloudT> empty;
  clouds_.clear();
  clouds_.swap(empty);
  load_cloud_index_ = 0;
  clouds_size_ = 0;
  current_cloud_index_ = -1;

  float cutAngle = -1.0;
  int direction = -1;
  int version = -1;

  ButtonsEnabled(false);

  lidar_ = new itd_lidar::lidar_driver::Driver(
    config.ip, config.groupIp, config.port, config.model,
    config.returnType, direction, version,
    config.correctionFilesPath, cutAngle,
    boost::bind(&PointCloudPlayer::LidarCallback, this, _1, _2),
    config.pcapFilePath);
  lidar_->Start();
}

void PointCloudPlayer::ButtonsEnabled(const bool &isEnable) {
  ui_->saveBtn->setEnabled(isEnable);
  // ui_->playBtn->setEnabled(isEnable);
  // ui_->pauseBtn->setEnabled(isEnable);
  ui_->nextBtn->setEnabled(isEnable);
  ui_->previousBtn->setEnabled(isEnable);
  // ui_->repeatBtn->setEnabled(isEnable);
  ui_->playSlider->setEnabled(isEnable);
  ui_->frameNb->setEnabled(isEnable);
  // ui_->playSpeedComboBox->setEnabled(isEnable);
}

void PointCloudPlayer::LidarCallback(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI>> cloud, int timestamp) {

  if (timestamp == -10) {
    ui_->playSlider->setMaximum(load_cloud_index_);
    ui_->playSlider->setMinimum(1);
    current_cloud_index_ = 0;
    clouds_size_ = load_cloud_index_;
    ButtonsEnabled(true);
    UpdatePushButtons(true);
    viewer_->removeAllPointClouds();
    viewer_->addPointCloud<pcl::PointXYZI>(clouds_[current_cloud_index_].makeShared(), "cloud");
    UpdateDisplay(current_cloud_index_);
    return;
  }

  clouds_[load_cloud_index_] = *cloud;
  load_cloud_index_++;
}
