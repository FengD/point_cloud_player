/********************************************************************************
** Form generated from reading UI file 'addlidardialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.7
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ADDLIDARDIALOG_H
#define UI_ADDLIDARDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AddLidarDialog
{
public:
    QDialogButtonBox *buttonBox;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *ipInput;
    QLabel *returnTypeLabel;
    QLabel *lidarModelLabel;
    QGridLayout *gridLayout_2;
    QRadioButton *pandar40RadioButton;
    QRadioButton *vlp16RadioButton;
    QGridLayout *gridLayout_3;
    QRadioButton *singleRadioButton;
    QRadioButton *dualRadioButton;
    QLabel *portLabel;
    QLabel *ipLabel;
    QLineEdit *portInput;
    QLabel *nameLabel;
    QLineEdit *nameInput;
    QLabel *correctionFileLabel;
    QVBoxLayout *verticalLayout;
    QPushButton *selectFileButton;
    QLabel *correctionFileInput;

    void setupUi(QDialog *AddLidarDialog)
    {
        if (AddLidarDialog->objectName().isEmpty())
            AddLidarDialog->setObjectName(QString::fromUtf8("AddLidarDialog"));
        AddLidarDialog->resize(613, 333);
        buttonBox = new QDialogButtonBox(AddLidarDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(30, 290, 561, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        gridLayoutWidget = new QWidget(AddLidarDialog);
        gridLayoutWidget->setObjectName(QString::fromUtf8("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(20, 10, 571, 271));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        ipInput = new QLineEdit(gridLayoutWidget);
        ipInput->setObjectName(QString::fromUtf8("ipInput"));

        gridLayout->addWidget(ipInput, 2, 1, 1, 1);

        returnTypeLabel = new QLabel(gridLayoutWidget);
        returnTypeLabel->setObjectName(QString::fromUtf8("returnTypeLabel"));

        gridLayout->addWidget(returnTypeLabel, 5, 0, 1, 1);

        lidarModelLabel = new QLabel(gridLayoutWidget);
        lidarModelLabel->setObjectName(QString::fromUtf8("lidarModelLabel"));

        gridLayout->addWidget(lidarModelLabel, 4, 0, 1, 1);

        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pandar40RadioButton = new QRadioButton(gridLayoutWidget);
        pandar40RadioButton->setObjectName(QString::fromUtf8("pandar40RadioButton"));

        gridLayout_2->addWidget(pandar40RadioButton, 2, 0, 1, 1);

        vlp16RadioButton = new QRadioButton(gridLayoutWidget);
        vlp16RadioButton->setObjectName(QString::fromUtf8("vlp16RadioButton"));

        gridLayout_2->addWidget(vlp16RadioButton, 0, 0, 1, 1);


        gridLayout->addLayout(gridLayout_2, 4, 1, 1, 1);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        singleRadioButton = new QRadioButton(gridLayoutWidget);
        singleRadioButton->setObjectName(QString::fromUtf8("singleRadioButton"));

        gridLayout_3->addWidget(singleRadioButton, 1, 1, 1, 1);

        dualRadioButton = new QRadioButton(gridLayoutWidget);
        dualRadioButton->setObjectName(QString::fromUtf8("dualRadioButton"));

        gridLayout_3->addWidget(dualRadioButton, 2, 1, 1, 1);


        gridLayout->addLayout(gridLayout_3, 5, 1, 1, 1);

        portLabel = new QLabel(gridLayoutWidget);
        portLabel->setObjectName(QString::fromUtf8("portLabel"));

        gridLayout->addWidget(portLabel, 3, 0, 1, 1);

        ipLabel = new QLabel(gridLayoutWidget);
        ipLabel->setObjectName(QString::fromUtf8("ipLabel"));

        gridLayout->addWidget(ipLabel, 2, 0, 1, 1);

        portInput = new QLineEdit(gridLayoutWidget);
        portInput->setObjectName(QString::fromUtf8("portInput"));

        gridLayout->addWidget(portInput, 3, 1, 1, 1);

        nameLabel = new QLabel(gridLayoutWidget);
        nameLabel->setObjectName(QString::fromUtf8("nameLabel"));

        gridLayout->addWidget(nameLabel, 0, 0, 1, 1);

        nameInput = new QLineEdit(gridLayoutWidget);
        nameInput->setObjectName(QString::fromUtf8("nameInput"));

        gridLayout->addWidget(nameInput, 0, 1, 1, 1);

        correctionFileLabel = new QLabel(gridLayoutWidget);
        correctionFileLabel->setObjectName(QString::fromUtf8("correctionFileLabel"));

        gridLayout->addWidget(correctionFileLabel, 6, 0, 1, 1);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        selectFileButton = new QPushButton(gridLayoutWidget);
        selectFileButton->setObjectName(QString::fromUtf8("selectFileButton"));

        verticalLayout->addWidget(selectFileButton);

        correctionFileInput = new QLabel(gridLayoutWidget);
        correctionFileInput->setObjectName(QString::fromUtf8("correctionFileInput"));

        verticalLayout->addWidget(correctionFileInput);


        gridLayout->addLayout(verticalLayout, 6, 1, 1, 1);


        retranslateUi(AddLidarDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), AddLidarDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), AddLidarDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(AddLidarDialog);
    } // setupUi

    void retranslateUi(QDialog *AddLidarDialog)
    {
        AddLidarDialog->setWindowTitle(QApplication::translate("AddLidarDialog", "Dialog", 0, QApplication::UnicodeUTF8));
        returnTypeLabel->setText(QApplication::translate("AddLidarDialog", "Return Type", 0, QApplication::UnicodeUTF8));
        lidarModelLabel->setText(QApplication::translate("AddLidarDialog", "Lidar Model", 0, QApplication::UnicodeUTF8));
        pandar40RadioButton->setText(QApplication::translate("AddLidarDialog", "P40P", 0, QApplication::UnicodeUTF8));
        vlp16RadioButton->setText(QApplication::translate("AddLidarDialog", "VLP16", 0, QApplication::UnicodeUTF8));
        singleRadioButton->setText(QApplication::translate("AddLidarDialog", "single", 0, QApplication::UnicodeUTF8));
        dualRadioButton->setText(QApplication::translate("AddLidarDialog", "dual", 0, QApplication::UnicodeUTF8));
        portLabel->setText(QApplication::translate("AddLidarDialog", "PORT", 0, QApplication::UnicodeUTF8));
        ipLabel->setText(QApplication::translate("AddLidarDialog", "IP", 0, QApplication::UnicodeUTF8));
        nameLabel->setText(QApplication::translate("AddLidarDialog", "name", 0, QApplication::UnicodeUTF8));
        correctionFileLabel->setText(QApplication::translate("AddLidarDialog", "Correction File", 0, QApplication::UnicodeUTF8));
        selectFileButton->setText(QApplication::translate("AddLidarDialog", "Select", 0, QApplication::UnicodeUTF8));
        correctionFileInput->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class AddLidarDialog: public Ui_AddLidarDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ADDLIDARDIALOG_H
