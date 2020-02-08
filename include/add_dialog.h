// Copyright (C) 2020 Feng DING, Hirain
// License: Modified BSD Software License Agreement

#ifndef ADDLIDARDIALOG_H
#define ADDLIDARDIALOG_H

#include <QDialog>
#include <QButtonGroup>
#include <string>
#include <iostream>

namespace Ui {
class Add;
}

struct CLidarConfig {
  int mode;
  std::string pcapFilePath;
  std::string ip;
  std::string groupIp;
  int port;
  std::string model;
  int returnType;
  std::vector<std::string> correctionFilesPath;
  CLidarConfig& operator= (const CLidarConfig& config) {
    mode = config.mode;
    pcapFilePath = config.pcapFilePath;
    ip = config.ip;
    groupIp = config.groupIp;
    port = config.port;
    model = config.model;
    returnType = config.returnType;
    correctionFilesPath = config.correctionFilesPath;
    return *this;
  }

  friend std::ostream &operator<< (std::ostream &output,
                                   const CLidarConfig &c) {
    output << "mode: " << c.mode << std::endl;
    output << "pcapFilePath: " << c.pcapFilePath << std::endl;
    output << "ip: " << c.ip << std::endl;
    output << "groupIp: " << c.groupIp << std::endl;
    output << "port: " << c.port << std::endl;
    output << "model: " << c.model << std::endl;
    output << "returnType: " << c.returnType;
    output << "correctionFilesPath: ";
    for (size_t i = 0; i < c.correctionFilesPath.size(); i++) {
      output << c.correctionFilesPath[i] << "; ";
    }
    return output;
  }

  CLidarConfig() {
    memset(this, 0, sizeof(CLidarConfig));
  }
};

class AddDialog : public QDialog {
  Q_OBJECT

 private:
  Ui::Add *ui;
  QButtonGroup *mode_group_;
  CLidarConfig lidar_config_;

  void InitParam();
  void CreateLidarModelBox();
  void CreateReturnTypeComboBox();
  void InitSlotConnect();

 public:
  explicit AddDialog(QWidget *parent = 0);
  ~AddDialog();
  CLidarConfig GetLidarConfig();

  private slots:
  void ipUpdate(const QString &input);
  void portUpdate(const QString &input);
  void groupIpUpdate(const QString &input);
  void modeGroupButtonsClicked(const int &id);
  void selectFileButtonPressed();
  void pcapFileInputButtonPressed();
  void lidarModelBoxChanged(const QString &text);
  void returnTypeBoxChanged(const QString &text);
};


#endif  // ADDLIDARDIALOG_H
