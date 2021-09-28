/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.13.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_main_window
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QSlider *horizontalSlider_R;
    QSlider *horizontalSlider_G;
    QSlider *horizontalSlider_B;
    QLCDNumber *lcdNumber_R;
    QLCDNumber *lcdNumber_G;
    QLCDNumber *lcdNumber_B;
    QSlider *horizontalSlider_p;
    QLCDNumber *lcdNumber_p;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QPushButton *pushButton_random;

    void setupUi(QMainWindow *main_window)
    {
        if (main_window->objectName().isEmpty())
            main_window->setObjectName(QString::fromUtf8("main_window"));
        main_window->resize(966, 499);
        main_window->setMinimumSize(QSize(0, 0));
        main_window->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(main_window);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(300, 10, 640, 480));
        horizontalSlider_R = new QSlider(centralwidget);
        horizontalSlider_R->setObjectName(QString::fromUtf8("horizontalSlider_R"));
        horizontalSlider_R->setGeometry(QRect(30, 60, 160, 29));
        horizontalSlider_R->setMaximum(255);
        horizontalSlider_R->setValue(128);
        horizontalSlider_R->setOrientation(Qt::Horizontal);
        horizontalSlider_G = new QSlider(centralwidget);
        horizontalSlider_G->setObjectName(QString::fromUtf8("horizontalSlider_G"));
        horizontalSlider_G->setGeometry(QRect(30, 140, 160, 29));
        horizontalSlider_G->setMaximum(255);
        horizontalSlider_G->setValue(128);
        horizontalSlider_G->setOrientation(Qt::Horizontal);
        horizontalSlider_B = new QSlider(centralwidget);
        horizontalSlider_B->setObjectName(QString::fromUtf8("horizontalSlider_B"));
        horizontalSlider_B->setGeometry(QRect(30, 220, 160, 29));
        horizontalSlider_B->setMaximum(255);
        horizontalSlider_B->setValue(128);
        horizontalSlider_B->setOrientation(Qt::Horizontal);
        lcdNumber_R = new QLCDNumber(centralwidget);
        lcdNumber_R->setObjectName(QString::fromUtf8("lcdNumber_R"));
        lcdNumber_R->setGeometry(QRect(200, 50, 81, 41));
        lcdNumber_R->setDigitCount(3);
        lcdNumber_R->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_R->setProperty("intValue", QVariant(128));
        lcdNumber_G = new QLCDNumber(centralwidget);
        lcdNumber_G->setObjectName(QString::fromUtf8("lcdNumber_G"));
        lcdNumber_G->setGeometry(QRect(200, 130, 81, 41));
        lcdNumber_G->setDigitCount(3);
        lcdNumber_G->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_G->setProperty("intValue", QVariant(128));
        lcdNumber_B = new QLCDNumber(centralwidget);
        lcdNumber_B->setObjectName(QString::fromUtf8("lcdNumber_B"));
        lcdNumber_B->setGeometry(QRect(200, 210, 81, 41));
        lcdNumber_B->setDigitCount(3);
        lcdNumber_B->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_B->setProperty("intValue", QVariant(128));
        horizontalSlider_p = new QSlider(centralwidget);
        horizontalSlider_p->setObjectName(QString::fromUtf8("horizontalSlider_p"));
        horizontalSlider_p->setGeometry(QRect(30, 320, 160, 29));
        horizontalSlider_p->setMinimum(1);
        horizontalSlider_p->setMaximum(6);
        horizontalSlider_p->setValue(2);
        horizontalSlider_p->setOrientation(Qt::Horizontal);
        lcdNumber_p = new QLCDNumber(centralwidget);
        lcdNumber_p->setObjectName(QString::fromUtf8("lcdNumber_p"));
        lcdNumber_p->setGeometry(QRect(200, 310, 81, 41));
        lcdNumber_p->setDigitCount(1);
        lcdNumber_p->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_p->setProperty("intValue", QVariant(2));
        label = new QLabel(centralwidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(30, 20, 191, 31));
        QFont font;
        font.setPointSize(16);
        font.setBold(false);
        font.setItalic(false);
        font.setWeight(50);
        label->setFont(font);
        label_2 = new QLabel(centralwidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(30, 100, 191, 31));
        label_2->setFont(font);
        label_3 = new QLabel(centralwidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(30, 190, 191, 31));
        label_3->setFont(font);
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(30, 280, 141, 31));
        label_4->setFont(font);
        pushButton_random = new QPushButton(centralwidget);
        pushButton_random->setObjectName(QString::fromUtf8("pushButton_random"));
        pushButton_random->setGeometry(QRect(40, 370, 201, 81));
        main_window->setCentralWidget(centralwidget);

        retranslateUi(main_window);
        QObject::connect(horizontalSlider_R, SIGNAL(sliderMoved(int)), lcdNumber_R, SLOT(display(int)));
        QObject::connect(horizontalSlider_G, SIGNAL(sliderMoved(int)), lcdNumber_G, SLOT(display(int)));
        QObject::connect(horizontalSlider_B, SIGNAL(sliderMoved(int)), lcdNumber_B, SLOT(display(int)));
        QObject::connect(horizontalSlider_p, SIGNAL(sliderMoved(int)), lcdNumber_p, SLOT(display(int)));

        QMetaObject::connectSlotsByName(main_window);
    } // setupUi

    void retranslateUi(QMainWindow *main_window)
    {
        main_window->setWindowTitle(QCoreApplication::translate("main_window", "PCLViewer", nullptr));
        label->setText(QCoreApplication::translate("main_window", "\347\272\242\350\211\262", nullptr));
        label_2->setText(QCoreApplication::translate("main_window", "\347\273\277\350\211\262", nullptr));
        label_3->setText(QCoreApplication::translate("main_window", "\350\223\235\350\211\262", nullptr));
        label_4->setText(QCoreApplication::translate("main_window", "\347\202\271\345\244\247\345\260\217", nullptr));
        pushButton_random->setText(QCoreApplication::translate("main_window", "\351\232\217\346\234\272\351\242\234\350\211\262", nullptr));
    } // retranslateUi

};

namespace Ui {
    class main_window: public Ui_main_window {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H
