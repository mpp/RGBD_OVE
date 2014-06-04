/********************************************************************************
** Form generated from reading UI file 'pclwindow.ui'
**
** Created: Tue Jun 3 22:04:59 2014
**      by: Qt User Interface Compiler version 4.8.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLWINDOW_H
#define UI_PCLWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QFrame>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QRadioButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_Pclwindow
{
public:
    QAction *exitAction;
    QAction *openFile0Action;
    QAction *openFile1Action;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QFrame *line_2;
    QHBoxLayout *cloudSelectionLayout;
    QRadioButton *cloud2Radio;
    QRadioButton *cloud3Radio;
    QRadioButton *cloud1Radio;
    QRadioButton *cloud0Radio;
    QVTKWidget *widget;
    QLabel *label;
    QGridLayout *buttonsLayout;
    QPushButton *yRIncButton;
    QPushButton *xRIncButton;
    QPushButton *yTIncButton;
    QPushButton *zTIncButton;
    QPushButton *zTDecButton;
    QPushButton *xRDecButton;
    QLabel *rotationStepLabel;
    QPushButton *zRDecButton;
    QLineEdit *rotationStepInput;
    QLineEdit *translationStepInput;
    QPushButton *yTDecButton;
    QLabel *translationStepLabel;
    QLabel *RotationLabel;
    QSpacerItem *horizontalSpacer;
    QLabel *translationLabel;
    QPushButton *xTIncButton;
    QPushButton *zRIncButton;
    QPushButton *yRDecButton;
    QPushButton *xTDecButton;
    QFrame *line;
    QLabel *label_2;
    QMenuBar *menuBar;
    QMenu *menu;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Pclwindow)
    {
        if (Pclwindow->objectName().isEmpty())
            Pclwindow->setObjectName(QString::fromUtf8("Pclwindow"));
        Pclwindow->resize(619, 560);
        exitAction = new QAction(Pclwindow);
        exitAction->setObjectName(QString::fromUtf8("exitAction"));
        openFile0Action = new QAction(Pclwindow);
        openFile0Action->setObjectName(QString::fromUtf8("openFile0Action"));
        openFile1Action = new QAction(Pclwindow);
        openFile1Action->setObjectName(QString::fromUtf8("openFile1Action"));
        centralWidget = new QWidget(Pclwindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QString::fromUtf8("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line_2, 6, 0, 1, 1);

        cloudSelectionLayout = new QHBoxLayout();
        cloudSelectionLayout->setSpacing(6);
        cloudSelectionLayout->setObjectName(QString::fromUtf8("cloudSelectionLayout"));
        cloudSelectionLayout->setContentsMargins(5, 5, 5, 5);
        cloud2Radio = new QRadioButton(centralWidget);
        cloud2Radio->setObjectName(QString::fromUtf8("cloud2Radio"));

        cloudSelectionLayout->addWidget(cloud2Radio);

        cloud3Radio = new QRadioButton(centralWidget);
        cloud3Radio->setObjectName(QString::fromUtf8("cloud3Radio"));

        cloudSelectionLayout->addWidget(cloud3Radio);

        cloud1Radio = new QRadioButton(centralWidget);
        cloud1Radio->setObjectName(QString::fromUtf8("cloud1Radio"));

        cloudSelectionLayout->addWidget(cloud1Radio);

        cloud0Radio = new QRadioButton(centralWidget);
        cloud0Radio->setObjectName(QString::fromUtf8("cloud0Radio"));

        cloudSelectionLayout->addWidget(cloud0Radio);


        gridLayout->addLayout(cloudSelectionLayout, 4, 0, 2, 1);

        widget = new QVTKWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(widget->sizePolicy().hasHeightForWidth());
        widget->setSizePolicy(sizePolicy);
        widget->setMinimumSize(QSize(601, 271));

        gridLayout->addWidget(widget, 0, 0, 1, 1);

        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);

        gridLayout->addWidget(label, 3, 0, 1, 1);

        buttonsLayout = new QGridLayout();
        buttonsLayout->setSpacing(6);
        buttonsLayout->setObjectName(QString::fromUtf8("buttonsLayout"));
        buttonsLayout->setSizeConstraint(QLayout::SetDefaultConstraint);
        buttonsLayout->setContentsMargins(5, 5, 5, 5);
        yRIncButton = new QPushButton(centralWidget);
        yRIncButton->setObjectName(QString::fromUtf8("yRIncButton"));

        buttonsLayout->addWidget(yRIncButton, 1, 5, 1, 1);

        xRIncButton = new QPushButton(centralWidget);
        xRIncButton->setObjectName(QString::fromUtf8("xRIncButton"));

        buttonsLayout->addWidget(xRIncButton, 1, 4, 1, 1);

        yTIncButton = new QPushButton(centralWidget);
        yTIncButton->setObjectName(QString::fromUtf8("yTIncButton"));

        buttonsLayout->addWidget(yTIncButton, 1, 1, 1, 1);

        zTIncButton = new QPushButton(centralWidget);
        zTIncButton->setObjectName(QString::fromUtf8("zTIncButton"));

        buttonsLayout->addWidget(zTIncButton, 1, 2, 1, 1);

        zTDecButton = new QPushButton(centralWidget);
        zTDecButton->setObjectName(QString::fromUtf8("zTDecButton"));

        buttonsLayout->addWidget(zTDecButton, 2, 2, 1, 1);

        xRDecButton = new QPushButton(centralWidget);
        xRDecButton->setObjectName(QString::fromUtf8("xRDecButton"));

        buttonsLayout->addWidget(xRDecButton, 2, 4, 1, 1);

        rotationStepLabel = new QLabel(centralWidget);
        rotationStepLabel->setObjectName(QString::fromUtf8("rotationStepLabel"));
        rotationStepLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        buttonsLayout->addWidget(rotationStepLabel, 0, 5, 1, 1);

        zRDecButton = new QPushButton(centralWidget);
        zRDecButton->setObjectName(QString::fromUtf8("zRDecButton"));

        buttonsLayout->addWidget(zRDecButton, 2, 6, 1, 1);

        rotationStepInput = new QLineEdit(centralWidget);
        rotationStepInput->setObjectName(QString::fromUtf8("rotationStepInput"));
        rotationStepInput->setMaximumSize(QSize(100, 30));

        buttonsLayout->addWidget(rotationStepInput, 0, 6, 1, 1);

        translationStepInput = new QLineEdit(centralWidget);
        translationStepInput->setObjectName(QString::fromUtf8("translationStepInput"));
        translationStepInput->setMaximumSize(QSize(100, 30));

        buttonsLayout->addWidget(translationStepInput, 0, 2, 1, 1);

        yTDecButton = new QPushButton(centralWidget);
        yTDecButton->setObjectName(QString::fromUtf8("yTDecButton"));

        buttonsLayout->addWidget(yTDecButton, 2, 1, 1, 1);

        translationStepLabel = new QLabel(centralWidget);
        translationStepLabel->setObjectName(QString::fromUtf8("translationStepLabel"));
        translationStepLabel->setLayoutDirection(Qt::RightToLeft);
        translationStepLabel->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        buttonsLayout->addWidget(translationStepLabel, 0, 1, 1, 1);

        RotationLabel = new QLabel(centralWidget);
        RotationLabel->setObjectName(QString::fromUtf8("RotationLabel"));

        buttonsLayout->addWidget(RotationLabel, 0, 4, 1, 1);

        horizontalSpacer = new QSpacerItem(20, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        buttonsLayout->addItem(horizontalSpacer, 0, 3, 1, 1);

        translationLabel = new QLabel(centralWidget);
        translationLabel->setObjectName(QString::fromUtf8("translationLabel"));

        buttonsLayout->addWidget(translationLabel, 0, 0, 1, 1);

        xTIncButton = new QPushButton(centralWidget);
        xTIncButton->setObjectName(QString::fromUtf8("xTIncButton"));

        buttonsLayout->addWidget(xTIncButton, 1, 0, 1, 1);

        zRIncButton = new QPushButton(centralWidget);
        zRIncButton->setObjectName(QString::fromUtf8("zRIncButton"));

        buttonsLayout->addWidget(zRIncButton, 1, 6, 1, 1);

        yRDecButton = new QPushButton(centralWidget);
        yRDecButton->setObjectName(QString::fromUtf8("yRDecButton"));

        buttonsLayout->addWidget(yRDecButton, 2, 5, 1, 1);

        xTDecButton = new QPushButton(centralWidget);
        xTDecButton->setObjectName(QString::fromUtf8("xTDecButton"));

        buttonsLayout->addWidget(xTDecButton, 2, 0, 1, 1);


        gridLayout->addLayout(buttonsLayout, 8, 0, 1, 1);

        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        gridLayout->addWidget(line, 2, 0, 1, 1);

        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setFont(font);

        gridLayout->addWidget(label_2, 7, 0, 1, 1);

        Pclwindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(Pclwindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 619, 25));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        Pclwindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(Pclwindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        Pclwindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(Pclwindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        Pclwindow->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menu->addAction(openFile0Action);
        menu->addAction(openFile1Action);
        menu->addSeparator();
        menu->addAction(exitAction);

        retranslateUi(Pclwindow);

        QMetaObject::connectSlotsByName(Pclwindow);
    } // setupUi

    void retranslateUi(QMainWindow *Pclwindow)
    {
        Pclwindow->setWindowTitle(QApplication::translate("Pclwindow", "Pclwindow", 0, QApplication::UnicodeUTF8));
        exitAction->setText(QApplication::translate("Pclwindow", "Exit", 0, QApplication::UnicodeUTF8));
        openFile0Action->setText(QApplication::translate("Pclwindow", "Open cam-0 file", 0, QApplication::UnicodeUTF8));
        openFile1Action->setText(QApplication::translate("Pclwindow", "Open cam-1 file", 0, QApplication::UnicodeUTF8));
        cloud2Radio->setText(QApplication::translate("Pclwindow", "camera 0", 0, QApplication::UnicodeUTF8));
        cloud3Radio->setText(QApplication::translate("Pclwindow", "camera 1", 0, QApplication::UnicodeUTF8));
        cloud1Radio->setText(QApplication::translate("Pclwindow", "camera 2", 0, QApplication::UnicodeUTF8));
        cloud0Radio->setText(QApplication::translate("Pclwindow", "camera 3", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Pclwindow", "Select the camera's cloud to edit:", 0, QApplication::UnicodeUTF8));
        yRIncButton->setText(QApplication::translate("Pclwindow", "+ y", 0, QApplication::UnicodeUTF8));
        xRIncButton->setText(QApplication::translate("Pclwindow", "+ x", 0, QApplication::UnicodeUTF8));
        yTIncButton->setText(QApplication::translate("Pclwindow", "+ y", 0, QApplication::UnicodeUTF8));
        zTIncButton->setText(QApplication::translate("Pclwindow", "+ z", 0, QApplication::UnicodeUTF8));
        zTDecButton->setText(QApplication::translate("Pclwindow", "- z", 0, QApplication::UnicodeUTF8));
        xRDecButton->setText(QApplication::translate("Pclwindow", "- x", 0, QApplication::UnicodeUTF8));
        rotationStepLabel->setText(QApplication::translate("Pclwindow", "step (rad)", 0, QApplication::UnicodeUTF8));
        zRDecButton->setText(QApplication::translate("Pclwindow", "- z", 0, QApplication::UnicodeUTF8));
        yTDecButton->setText(QApplication::translate("Pclwindow", "- y", 0, QApplication::UnicodeUTF8));
        translationStepLabel->setText(QApplication::translate("Pclwindow", "step (m)", 0, QApplication::UnicodeUTF8));
        RotationLabel->setText(QApplication::translate("Pclwindow", "Rotation", 0, QApplication::UnicodeUTF8));
        translationLabel->setText(QApplication::translate("Pclwindow", "Translation", 0, QApplication::UnicodeUTF8));
        xTIncButton->setText(QApplication::translate("Pclwindow", "+ x", 0, QApplication::UnicodeUTF8));
        zRIncButton->setText(QApplication::translate("Pclwindow", "+ z", 0, QApplication::UnicodeUTF8));
        yRDecButton->setText(QApplication::translate("Pclwindow", "- y", 0, QApplication::UnicodeUTF8));
        xTDecButton->setText(QApplication::translate("Pclwindow", "- x", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("Pclwindow", "Edit controls:", 0, QApplication::UnicodeUTF8));
        menu->setTitle(QApplication::translate("Pclwindow", "File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Pclwindow: public Ui_Pclwindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLWINDOW_H
