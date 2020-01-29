#include "add_dialog.h"
#include <string.h>
#include <iostream>
#include <QFileDialog>
#include "ui_add_dialog.h"

AddDialog::AddDialog(QWidget *parent) :
  QDialog(parent),
  ui(new Ui::Add) {
  ui->setupUi(this);

  modeGroup = new QButtonGroup();
  onlineRadBtn = findChild<QRadioButton*>("onlineRadBtn");
  offlineRadBtn = findChild<QRadioButton*>("offlineRadBtn");

  correctionFileInput = findChild<QLabel*>("correctionFileInput");
  filePath = findChild<QLabel*>("filePath");

  selectFileButton = findChild<QPushButton*>("selectFileButton");
  pcapFileInputBtn = findChild<QPushButton*>("pcapFileInputBtn");

  lidarModelBox = findChild<QComboBox*>("lidarModelBox");
  returnTypeBox = findChild<QComboBox*>("returnTypeBox");

  ipInput = findChild<QLineEdit*>("ipInput");
  portInput = findChild<QLineEdit*>("portInput");

  init();
  initConnection();
}

AddDialog::~AddDialog() {
  delete modeGroup;
  delete onlineRadBtn;
  delete offlineRadBtn;
  delete ipInput;
  delete portInput;
  delete correctionFileInput;
  delete filePath;
  delete selectFileButton;
  delete pcapFileInputBtn;
  delete lidarModelBox;
  delete returnTypeBox;
  delete ui;
}

void AddDialog::init() {
  modeGroup->addButton(offlineRadBtn, 2);
  modeGroup->addButton(onlineRadBtn, 1);
  offlineRadBtn->setChecked(true);

  portInput->setValidator(new QIntValidator(portInput));
  ipInput->setInputMask("000.000.000.000");

  correctionFileInput->setEnabled(false);
  selectFileButton->setEnabled(false);

  lidarConfig.mode = 2;
  lidarConfig.ip = "192.168.8.201";
  lidarConfig.model = "VLP16";
  lidarConfig.port = 2368;
  lidarConfig.returnType = 0;
  lidarConfig.correctionFilePath = " ";

  createLidarModelBox();
  createReturnTypeComboBox();
}

void AddDialog::createLidarModelBox() {
  QStringList QList;
  QList.clear();
  QList << tr("VLP16")
        << tr("VLP32MR")
        << tr("RS32")
        << tr("P40P")
        << tr("InnovizPro");
  lidarModelBox->clear();
  lidarModelBox->addItems(QList);
  lidarModelBox->setCurrentIndex(0);
}

void AddDialog::createReturnTypeComboBox() {
  QStringList QList;
  QList.clear();
  QList << tr("Single")
        << tr("Dual");
  returnTypeBox->clear();
  returnTypeBox->addItems(QList);
  returnTypeBox->setCurrentIndex(0);
}


void AddDialog::initConnection() {
  connect (ipInput, SIGNAL (textChanged(const QString &)), this, SLOT (ipUpdate(const QString &)));
  connect (portInput, SIGNAL (textChanged(const QString &)), this, SLOT (portUpdate(const QString &)));

  connect(modeGroup, SIGNAL(buttonClicked(const int &)), this, SLOT(modeGroupButtonsClicked(const int &)));
  connect (selectFileButton,  SIGNAL (clicked ()), this, SLOT (selectFileButtonPressed ()));
  connect (pcapFileInputBtn,  SIGNAL (clicked ()), this, SLOT (pcapFileInputButtonPressed ()));

  connect(lidarModelBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(lidarModelBoxChanged(const QString &)));
  connect(returnTypeBox, SIGNAL(currentIndexChanged(const QString &)),
          this, SLOT(returnTpypeBoxChanged(const QString &)));

}

void AddDialog::ipUpdate(const QString &input) {
  lidarConfig.ip = input.toLatin1().data();
}

void AddDialog::portUpdate(const QString &input) {
  lidarConfig.port = input.toInt();
}

CLidarConfig AddDialog::getLidarConfig() {
  return lidarConfig;
}

void AddDialog::modeGroupButtonsClicked(const int &id) {
  if (id == 1) {
    filePath->setEnabled(false);
    pcapFileInputBtn->setEnabled(false);
  } else if (id == 2) {
    filePath->setEnabled(true);
    pcapFileInputBtn->setEnabled(true);
  }
  lidarConfig.mode = id;
}

void AddDialog::selectFileButtonPressed() {
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select"), "/", tr("Files (*.csv)"));
  correctionFileInput->setText(fileName);
  lidarConfig.correctionFilePath = fileName.toStdString();
}

void AddDialog::pcapFileInputButtonPressed() {
  QString fileName = QFileDialog::getOpenFileName(this,
    tr("Select"), "/", tr("Files (*.pcap)"));
  filePath->setText(fileName);
  lidarConfig.pcapFilePath = fileName.toStdString();
}

void AddDialog::lidarModelBoxChanged(const QString &text) {
  std::string model = text.toLatin1().data();
  if (model == "VLP16" || model == "VLP32MR") {
    correctionFileInput->setEnabled(false);
    selectFileButton->setEnabled(false);
  } else {
    correctionFileInput->setEnabled(true);
    selectFileButton->setEnabled(true);
  }
  lidarConfig.model = model;
}

void AddDialog::returnTpypeBoxChanged(const QString &text) {
  (void) text;
  lidarConfig.returnType = returnTypeBox->currentIndex();
}
