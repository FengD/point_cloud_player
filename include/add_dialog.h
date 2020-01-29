#ifndef ADDLIDARDIALOG_H
#define ADDLIDARDIALOG_H

#include <QWidget>
#include <QDebug>
#include <QPushButton>
#include <QDialog>
#include <QLineEdit>
#include <QRadioButton>
#include <QButtonGroup>
#include <QLabel>
#include <QComboBox>
#include <string>
#include <iostream>

namespace Ui {
class Add;
}

struct CLidarConfig {
  int mode;
  std::string pcapFilePath;
  std::string ip;
  int port;
  std::string model;
  int returnType;
  std::string correctionFilePath;
  CLidarConfig& operator= (const CLidarConfig& config) {
    mode = config.mode;
    pcapFilePath = config.pcapFilePath;
    ip = config.ip;
    port = config.port;
    model = config.model;
    returnType = config.returnType;
    correctionFilePath = config.correctionFilePath;
    return *this;
  }

  friend std::ostream &operator<<( std::ostream &output, const CLidarConfig &c ) {
    output << "mode: " << c.mode << std::endl;
    output << "pcapFilePath: " << c.pcapFilePath << std::endl;
    output << "ip: " << c.ip << std::endl;
    output << "port: " << c.port << std::endl;
    output << "model: " << c.model << std::endl;
    output << "returnType: " << c.returnType << std::endl;
    output << "correctionFilePath: " << c.correctionFilePath << std::endl;
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
    QButtonGroup *modeGroup;
    QRadioButton *onlineRadBtn, *offlineRadBtn;
    QLineEdit *ipInput, *portInput;

    QLabel *correctionFileInput, *filePath;
    QPushButton *selectFileButton, *pcapFileInputBtn;
    CLidarConfig lidarConfig;

    QComboBox *lidarModelBox, *returnTypeBox;

    void init();
    void createLidarModelBox();
    void createReturnTypeComboBox();
    void initConnection();

public:
    AddDialog(QWidget *parent = 0);
    ~AddDialog();
    CLidarConfig getLidarConfig();

private slots:
    void ipUpdate(const QString &input);
    void portUpdate(const QString &input);
    void modeGroupButtonsClicked(const int &id);
    void selectFileButtonPressed();
    void pcapFileInputButtonPressed();
    void lidarModelBoxChanged(const QString &text);
    void returnTpypeBoxChanged(const QString &text);
};


#endif // ADDLIDARDIALOG_H
