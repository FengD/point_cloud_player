// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#include "add_dialog.h"
#include <string.h>
#include <iostream>
#include <QFileDialog>
#include "ui_add_dialog.h"

AddDialog::AddDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Add) {
  ui->setupUi(this);
  mode_group_ = new QButtonGroup();
  CreateLidarModelBox();
  CreateReturnTypeComboBox();
  InitParam();
  InitSlotConnect();
}

AddDialog::~AddDialog() {
  delete mode_group_;
  delete ui;
}

void AddDialog::InitParam() {
  mode_group_->addButton(ui->offlineRadBtn, 2);
  mode_group_->addButton(ui->onlineRadBtn, 1);
  ui->offlineRadBtn->setChecked(true);

  ui->portInput->setValidator(new QIntValidator(ui->portInput));
  ui->ipInput->setInputMask("000.000.000.000");
  ui->groupIpInput->setInputMask("000.000.000.000");

  ui->correctionFileInput->setEnabled(false);
  ui->selectFileButton->setEnabled(false);

  lidar_config_.mode = 2;
  lidar_config_.ip = "192.168.8.201";
  lidar_config_.groupIp = "239.0.0.1";
  lidar_config_.model = "VLP16";
  lidar_config_.port = 2368;
  lidar_config_.returnType = 0;
  // lidar_config_.correctionFilePath = " ";
}

void AddDialog::CreateLidarModelBox() {
  QStringList QList;
  QList.clear();
  QList << tr("VLP16") << tr("VLP32MR") << tr("RS32") << tr("P40P")
        << tr("InnovizPro");
  ui->lidarModelBox->clear();
  ui->lidarModelBox->addItems(QList);
  ui->lidarModelBox->setCurrentIndex(0);
}

void AddDialog::CreateReturnTypeComboBox() {
  QStringList QList;
  QList.clear();
  QList << tr("Single")
        << tr("Dual");
  ui->returnTypeBox->clear();
  ui->returnTypeBox->addItems(QList);
  ui->returnTypeBox->setCurrentIndex(0);
}


void AddDialog::InitSlotConnect() {
  connect(ui->ipInput, SIGNAL (textChanged(const QString &)),
          this, SLOT (ipUpdate(const QString &)));
  connect(ui->portInput, SIGNAL (textChanged(const QString &)),
          this, SLOT (portUpdate(const QString &)));
  connect(ui->groupIpInput, SIGNAL (textChanged(const QString &)),
          this, SLOT (groupIpUpdate(const QString &)));
  connect(mode_group_, SIGNAL(buttonClicked(const int &)),
          this, SLOT(modeGroupButtonsClicked(const int &)));
  connect(ui->selectFileButton,  SIGNAL (clicked ()),
          this, SLOT (selectFileButtonPressed ()));
  connect(ui->pcapFileInputBtn,  SIGNAL (clicked ()),
          this, SLOT (pcapFileInputButtonPressed ()));
  connect(ui->lidarModelBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(lidarModelBoxChanged(const QString &)));
  connect(ui->returnTypeBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(returnTypeBoxChanged(const QString &)));
}

void AddDialog::ipUpdate(const QString &input) {
  lidar_config_.ip = input.toLatin1().data();
}

void AddDialog::portUpdate(const QString &input) {
  lidar_config_.port = input.toInt();
}

void AddDialog::groupIpUpdate(const QString &input) {
  lidar_config_.groupIp = input.toLatin1().data();
}

CLidarConfig AddDialog::GetLidarConfig() {
  return lidar_config_;
}

void AddDialog::modeGroupButtonsClicked(const int &id) {
  if (id == 1) {
    ui->filePath->setEnabled(false);
    ui->pcapFileInputBtn->setEnabled(false);
  } else if (id == 2) {
    ui->filePath->setEnabled(true);
    ui->pcapFileInputBtn->setEnabled(true);
  }
  lidar_config_.mode = id;
}

void AddDialog::selectFileButtonPressed() {
  QStringList fileNames = QFileDialog::getOpenFileNames(this,
    tr("Select"), "/", tr("Files (*.csv)"));
  QString files;
  for(int i = 0; i< fileNames.size();++i) {
    QString fileName = fileNames.at(i);
    files += fileName + ";";
    lidar_config_.correctionFilesPath.push_back(fileName.toStdString());
  }
  ui->correctionFileInput->setText(files);
}

void AddDialog::pcapFileInputButtonPressed() {
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select"), "/", tr("Files (*.pcap)"));
  ui->filePath->setText(fileName);
  lidar_config_.pcapFilePath = fileName.toStdString();
}

void AddDialog::lidarModelBoxChanged(const QString &text) {
  std::string model = text.toLatin1().data();
  if (model == "VLP16" || model == "VLP32MR") {
    ui->correctionFileInput->setEnabled(false);
    ui->selectFileButton->setEnabled(false);
  } else {
    ui->correctionFileInput->setEnabled(true);
    ui->selectFileButton->setEnabled(true);
  }
  lidar_config_.model = model;
}

void AddDialog::returnTypeBoxChanged(const QString &text) {
  (void) text;
  lidar_config_.returnType = ui->returnTypeBox->currentIndex();
}
