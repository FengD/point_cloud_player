/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLCDNumber>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionAdd_Lidar;
    QWidget *centralWidget;
    QWidget *gridLayoutWidget_4;
    QGridLayout *gridLayout_4;
    QSlider *greenSlider;
    QLCDNumber *blueDisplay;
    QLabel *blueLabel;
    QSlider *redSlider;
    QSlider *blueSlider;
    QLCDNumber *greenDisplay;
    QLCDNumber *redDisplay;
    QLabel *greenLabel;
    QLabel *redLabel;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QLCDNumber *pitchDisplay;
    QLabel *rollLable;
    QLabel *yawLabel;
    QLCDNumber *rollDisplay;
    QLabel *yLabel;
    QLabel *pitchLabel;
    QLabel *zLabel;
    QLCDNumber *yDisplay;
    QLCDNumber *yawDisplay;
    QLCDNumber *zDisplay;
    QLCDNumber *xDisplay;
    QSlider *ySlider;
    QSlider *zSlider;
    QSlider *rollSlider;
    QSlider *pitchSlider;
    QSlider *yawSlider;
    QSlider *xSlider;
    QLabel *xLable;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLCDNumber *pointSizeDisplay;
    QLabel *pointSizeLabel;
    QSlider *pointSizeSlider;
    QPushButton *fileGenerationButton;
    QWidget *formLayoutWidget;
    QFormLayout *formLayout;
    QLabel *label_1;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *currentLidarName;
    QLabel *currentLidarIp;
    QLabel *currentLidarPort;
    QLabel *label_4;
    QLabel *currentLidarModel;
    QLabel *label_6;
    QLabel *currentLidarType;
    QPushButton *connectButton;
    QPushButton *disconnectButton;
    QMenuBar *actionMenuBar;
    QMenu *menuFirst;
    QMenu *menuSecond;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(232, 716);
        actionAdd_Lidar = new QAction(MainWindow);
        actionAdd_Lidar->setObjectName(QString::fromUtf8("actionAdd_Lidar"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayoutWidget_4 = new QWidget(centralWidget);
        gridLayoutWidget_4->setObjectName(QString::fromUtf8("gridLayoutWidget_4"));
        gridLayoutWidget_4->setGeometry(QRect(10, 260, 201, 101));
        gridLayout_4 = new QGridLayout(gridLayoutWidget_4);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QString::fromUtf8("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        greenSlider = new QSlider(gridLayoutWidget_4);
        greenSlider->setObjectName(QString::fromUtf8("greenSlider"));
        greenSlider->setCursor(QCursor(Qt::PointingHandCursor));
        greenSlider->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(greenSlider, 2, 1, 1, 1);

        blueDisplay = new QLCDNumber(gridLayoutWidget_4);
        blueDisplay->setObjectName(QString::fromUtf8("blueDisplay"));

        gridLayout_4->addWidget(blueDisplay, 3, 2, 1, 1);

        blueLabel = new QLabel(gridLayoutWidget_4);
        blueLabel->setObjectName(QString::fromUtf8("blueLabel"));

        gridLayout_4->addWidget(blueLabel, 3, 0, 1, 1);

        redSlider = new QSlider(gridLayoutWidget_4);
        redSlider->setObjectName(QString::fromUtf8("redSlider"));
        redSlider->setCursor(QCursor(Qt::PointingHandCursor));
        redSlider->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(redSlider, 1, 1, 1, 1);

        blueSlider = new QSlider(gridLayoutWidget_4);
        blueSlider->setObjectName(QString::fromUtf8("blueSlider"));
        blueSlider->setCursor(QCursor(Qt::PointingHandCursor));
        blueSlider->setOrientation(Qt::Horizontal);

        gridLayout_4->addWidget(blueSlider, 3, 1, 1, 1);

        greenDisplay = new QLCDNumber(gridLayoutWidget_4);
        greenDisplay->setObjectName(QString::fromUtf8("greenDisplay"));

        gridLayout_4->addWidget(greenDisplay, 2, 2, 1, 1);

        redDisplay = new QLCDNumber(gridLayoutWidget_4);
        redDisplay->setObjectName(QString::fromUtf8("redDisplay"));

        gridLayout_4->addWidget(redDisplay, 1, 2, 1, 1);

        greenLabel = new QLabel(gridLayoutWidget_4);
        greenLabel->setObjectName(QString::fromUtf8("greenLabel"));

        gridLayout_4->addWidget(greenLabel, 2, 0, 1, 1);

        redLabel = new QLabel(gridLayoutWidget_4);
        redLabel->setObjectName(QString::fromUtf8("redLabel"));

        gridLayout_4->addWidget(redLabel, 1, 0, 1, 1);

        gridLayoutWidget_3 = new QWidget(centralWidget);
        gridLayoutWidget_3->setObjectName(QString::fromUtf8("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 10, 201, 231));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        pitchDisplay = new QLCDNumber(gridLayoutWidget_3);
        pitchDisplay->setObjectName(QString::fromUtf8("pitchDisplay"));

        gridLayout_3->addWidget(pitchDisplay, 4, 2, 1, 1);

        rollLable = new QLabel(gridLayoutWidget_3);
        rollLable->setObjectName(QString::fromUtf8("rollLable"));

        gridLayout_3->addWidget(rollLable, 3, 0, 1, 1);

        yawLabel = new QLabel(gridLayoutWidget_3);
        yawLabel->setObjectName(QString::fromUtf8("yawLabel"));

        gridLayout_3->addWidget(yawLabel, 5, 0, 1, 1);

        rollDisplay = new QLCDNumber(gridLayoutWidget_3);
        rollDisplay->setObjectName(QString::fromUtf8("rollDisplay"));

        gridLayout_3->addWidget(rollDisplay, 3, 2, 1, 1);

        yLabel = new QLabel(gridLayoutWidget_3);
        yLabel->setObjectName(QString::fromUtf8("yLabel"));

        gridLayout_3->addWidget(yLabel, 1, 0, 1, 1);

        pitchLabel = new QLabel(gridLayoutWidget_3);
        pitchLabel->setObjectName(QString::fromUtf8("pitchLabel"));

        gridLayout_3->addWidget(pitchLabel, 4, 0, 1, 1);

        zLabel = new QLabel(gridLayoutWidget_3);
        zLabel->setObjectName(QString::fromUtf8("zLabel"));

        gridLayout_3->addWidget(zLabel, 2, 0, 1, 1);

        yDisplay = new QLCDNumber(gridLayoutWidget_3);
        yDisplay->setObjectName(QString::fromUtf8("yDisplay"));

        gridLayout_3->addWidget(yDisplay, 1, 2, 1, 1);

        yawDisplay = new QLCDNumber(gridLayoutWidget_3);
        yawDisplay->setObjectName(QString::fromUtf8("yawDisplay"));

        gridLayout_3->addWidget(yawDisplay, 5, 2, 1, 1);

        zDisplay = new QLCDNumber(gridLayoutWidget_3);
        zDisplay->setObjectName(QString::fromUtf8("zDisplay"));

        gridLayout_3->addWidget(zDisplay, 2, 2, 1, 1);

        xDisplay = new QLCDNumber(gridLayoutWidget_3);
        xDisplay->setObjectName(QString::fromUtf8("xDisplay"));

        gridLayout_3->addWidget(xDisplay, 0, 2, 1, 1);

        ySlider = new QSlider(gridLayoutWidget_3);
        ySlider->setObjectName(QString::fromUtf8("ySlider"));
        ySlider->setCursor(QCursor(Qt::PointingHandCursor));
        ySlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(ySlider, 1, 1, 1, 1);

        zSlider = new QSlider(gridLayoutWidget_3);
        zSlider->setObjectName(QString::fromUtf8("zSlider"));
        zSlider->setCursor(QCursor(Qt::PointingHandCursor));
        zSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(zSlider, 2, 1, 1, 1);

        rollSlider = new QSlider(gridLayoutWidget_3);
        rollSlider->setObjectName(QString::fromUtf8("rollSlider"));
        rollSlider->setCursor(QCursor(Qt::PointingHandCursor));
        rollSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(rollSlider, 3, 1, 1, 1);

        pitchSlider = new QSlider(gridLayoutWidget_3);
        pitchSlider->setObjectName(QString::fromUtf8("pitchSlider"));
        pitchSlider->setCursor(QCursor(Qt::PointingHandCursor));
        pitchSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(pitchSlider, 4, 1, 1, 1);

        yawSlider = new QSlider(gridLayoutWidget_3);
        yawSlider->setObjectName(QString::fromUtf8("yawSlider"));
        yawSlider->setCursor(QCursor(Qt::PointingHandCursor));
        yawSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(yawSlider, 5, 1, 1, 1);

        xSlider = new QSlider(gridLayoutWidget_3);
        xSlider->setObjectName(QString::fromUtf8("xSlider"));
        xSlider->setCursor(QCursor(Qt::PointingHandCursor));
        xSlider->setOrientation(Qt::Horizontal);

        gridLayout_3->addWidget(xSlider, 0, 1, 1, 1);

        xLable = new QLabel(gridLayoutWidget_3);
        xLable->setObjectName(QString::fromUtf8("xLable"));

        gridLayout_3->addWidget(xLable, 0, 0, 1, 1);

        gridLayoutWidget = new QWidget(centralWidget);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 380, 201, 31));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        pointSizeDisplay = new QLCDNumber(gridLayoutWidget);
        pointSizeDisplay->setObjectName(QString::fromUtf8("pointSizeDisplay"));

        gridLayout->addWidget(pointSizeDisplay, 1, 2, 1, 1);

        pointSizeLabel = new QLabel(gridLayoutWidget);
        pointSizeLabel->setObjectName(QString::fromUtf8("pointSizeLabel"));

        gridLayout->addWidget(pointSizeLabel, 1, 0, 1, 1);

        pointSizeSlider = new QSlider(gridLayoutWidget);
        pointSizeSlider->setObjectName(QString::fromUtf8("pointSizeSlider"));
        pointSizeSlider->setCursor(QCursor(Qt::PointingHandCursor));
        pointSizeSlider->setOrientation(Qt::Horizontal);

        gridLayout->addWidget(pointSizeSlider, 1, 1, 1, 1);

        fileGenerationButton = new QPushButton(centralWidget);
        fileGenerationButton->setObjectName(QString::fromUtf8("fileGenerationButton"));
        fileGenerationButton->setGeometry(QRect(10, 570, 201, 41));
        formLayoutWidget = new QWidget(centralWidget);
        formLayoutWidget->setObjectName(QString::fromUtf8("formLayoutWidget"));
        formLayoutWidget->setGeometry(QRect(10, 439, 201, 111));
        formLayout = new QFormLayout(formLayoutWidget);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setContentsMargins(0, 0, 0, 0);
        label_1 = new QLabel(formLayoutWidget);
        label_1->setObjectName(QString::fromUtf8("label_1"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_1);

        label_2 = new QLabel(formLayoutWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        label_3 = new QLabel(formLayoutWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        currentLidarName = new QLabel(formLayoutWidget);
        currentLidarName->setObjectName(QString::fromUtf8("currentLidarName"));

        formLayout->setWidget(0, QFormLayout::FieldRole, currentLidarName);

        currentLidarIp = new QLabel(formLayoutWidget);
        currentLidarIp->setObjectName(QString::fromUtf8("currentLidarIp"));

        formLayout->setWidget(1, QFormLayout::FieldRole, currentLidarIp);

        currentLidarPort = new QLabel(formLayoutWidget);
        currentLidarPort->setObjectName(QString::fromUtf8("currentLidarPort"));

        formLayout->setWidget(2, QFormLayout::FieldRole, currentLidarPort);

        label_4 = new QLabel(formLayoutWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_4);

        currentLidarModel = new QLabel(formLayoutWidget);
        currentLidarModel->setObjectName(QString::fromUtf8("currentLidarModel"));

        formLayout->setWidget(3, QFormLayout::FieldRole, currentLidarModel);

        label_6 = new QLabel(formLayoutWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_6);

        currentLidarType = new QLabel(formLayoutWidget);
        currentLidarType->setObjectName(QString::fromUtf8("currentLidarType"));

        formLayout->setWidget(4, QFormLayout::FieldRole, currentLidarType);

        connectButton = new QPushButton(centralWidget);
        connectButton->setObjectName(QString::fromUtf8("connectButton"));
        connectButton->setGeometry(QRect(10, 620, 91, 31));
        disconnectButton = new QPushButton(centralWidget);
        disconnectButton->setObjectName(QString::fromUtf8("disconnectButton"));
        disconnectButton->setGeometry(QRect(120, 620, 91, 31));
        MainWindow->setCentralWidget(centralWidget);
        actionMenuBar = new QMenuBar(MainWindow);
        actionMenuBar->setObjectName(QString::fromUtf8("actionMenuBar"));
        actionMenuBar->setGeometry(QRect(0, 0, 232, 25));
        menuFirst = new QMenu(actionMenuBar);
        menuFirst->setObjectName(QString::fromUtf8("menuFirst"));
        menuSecond = new QMenu(actionMenuBar);
        menuSecond->setObjectName(QString::fromUtf8("menuSecond"));
        MainWindow->setMenuBar(actionMenuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        actionMenuBar->addAction(menuFirst->menuAction());
        actionMenuBar->addAction(menuSecond->menuAction());
        menuFirst->addAction(actionAdd_Lidar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionAdd_Lidar->setText(QApplication::translate("MainWindow", "Add Lidar", 0, QApplication::UnicodeUTF8));
        blueLabel->setText(QApplication::translate("MainWindow", "Blue", 0, QApplication::UnicodeUTF8));
        greenLabel->setText(QApplication::translate("MainWindow", "Green", 0, QApplication::UnicodeUTF8));
        redLabel->setText(QApplication::translate("MainWindow", "Red", 0, QApplication::UnicodeUTF8));
        rollLable->setText(QApplication::translate("MainWindow", "Roll", 0, QApplication::UnicodeUTF8));
        yawLabel->setText(QApplication::translate("MainWindow", "Yaw", 0, QApplication::UnicodeUTF8));
        yLabel->setText(QApplication::translate("MainWindow", "Y", 0, QApplication::UnicodeUTF8));
        pitchLabel->setText(QApplication::translate("MainWindow", "Pitch", 0, QApplication::UnicodeUTF8));
        zLabel->setText(QApplication::translate("MainWindow", "Z", 0, QApplication::UnicodeUTF8));
        xLable->setText(QApplication::translate("MainWindow", "X", 0, QApplication::UnicodeUTF8));
        pointSizeLabel->setText(QApplication::translate("MainWindow", "Point Size", 0, QApplication::UnicodeUTF8));
        fileGenerationButton->setText(QApplication::translate("MainWindow", "Generate Calibration File", 0, QApplication::UnicodeUTF8));
        label_1->setText(QApplication::translate("MainWindow", "Current", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Ip", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Port", 0, QApplication::UnicodeUTF8));
        currentLidarName->setText(QString());
        currentLidarIp->setText(QString());
        currentLidarPort->setText(QString());
        label_4->setText(QApplication::translate("MainWindow", "Model", 0, QApplication::UnicodeUTF8));
        currentLidarModel->setText(QString());
        label_6->setText(QApplication::translate("MainWindow", "Type", 0, QApplication::UnicodeUTF8));
        currentLidarType->setText(QString());
        connectButton->setText(QApplication::translate("MainWindow", "Connect", 0, QApplication::UnicodeUTF8));
        disconnectButton->setText(QApplication::translate("MainWindow", "Disconnect", 0, QApplication::UnicodeUTF8));
        menuFirst->setTitle(QApplication::translate("MainWindow", "Action", 0, QApplication::UnicodeUTF8));
        menuSecond->setTitle(QApplication::translate("MainWindow", "Lidar Select", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
