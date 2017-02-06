/********************************************************************************
** Form generated from reading UI file 'cabledrivenparallerobot.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CABLEDRIVENPARALLEROBOT_H
#define UI_CABLEDRIVENPARALLEROBOT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_CableDrivenParalleRobot
{
public:
    QAction *actionNew_Window;
    QAction *actionClose;
    QAction *actionFile;
    QAction *actionRobot;
    QAction *actionCopy;
    QAction *actionPaste;
    QAction *actionClear;
    QWidget *centralWidget;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QLabel *label_16;
    QWidget *layoutWidget;
    QGridLayout *gridLayout_2;
    QGridLayout *gridLayout;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLineEdit *anchorpoint1;
    QLabel *label_4;
    QLineEdit *anchorpoint2;
    QLineEdit *exitpoint2;
    QLabel *label_5;
    QLineEdit *anchorpoint3;
    QLabel *label_12;
    QLineEdit *exitpoint3;
    QLineEdit *exitpoint1;
    QLabel *label_7;
    QLineEdit *anchorpoint4;
    QLineEdit *exitpoint4;
    QLabel *label_8;
    QLineEdit *anchorpoint5;
    QLineEdit *exitpoint5;
    QLabel *label_9;
    QLineEdit *anchorpoint6;
    QLabel *label_10;
    QLineEdit *anchorpoint7;
    QLineEdit *exitpoint7;
    QLabel *label_11;
    QLineEdit *exitpoint8;
    QLineEdit *anchorpoint8;
    QLineEdit *exitpoint6;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_17;
    QLineEdit *CableMass;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_18;
    QLineEdit *CableRadius;
    QPushButton *pushButton_3;
    QLabel *label_19;
    QLabel *label_29;
    QLineEdit *CableMass_3;
    QWidget *layoutWidget3;
    QGridLayout *gridLayout_3;
    QLabel *label_24;
    QLineEdit *Inertialyy;
    QLabel *label_26;
    QLineEdit *Inertialyz;
    QLabel *label_27;
    QLineEdit *Inertialyz_2;
    QLabel *label_25;
    QLineEdit *Inertialxy;
    QLabel *label_6;
    QLineEdit *Inertialxx;
    QLabel *label_20;
    QLineEdit *Inertialxz;
    QLineEdit *Inertialxy_2;
    QLabel *label_21;
    QLabel *label_22;
    QLineEdit *Inertialxz_2;
    QLabel *label_23;
    QLineEdit *Inertialzz;
    QWidget *layoutWidget4;
    QGridLayout *gridLayout_4;
    QLabel *label_28;
    QHBoxLayout *horizontalLayout;
    QLabel *label_13;
    QLineEdit *endeffMass;
    QComboBox *endEffShape_2;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_14;
    QComboBox *endEffShape;
    QLabel *label_15;
    QWidget *layoutWidget5;
    QGridLayout *gridLayout_6;
    QLabel *label_31;
    QLabel *label_37;
    QLabel *label_30;
    QLineEdit *endeffRPY;
    QLineEdit *endeffPosition;
    QWidget *layoutWidget6;
    QGridLayout *gridLayout_9;
    QGridLayout *gridLayout_7;
    QComboBox *endEffShape_5;
    QHBoxLayout *horizontalLayout_8;
    QLabel *label_40;
    QComboBox *endEffShape_6;
    QHBoxLayout *horizontalLayout_7;
    QLabel *label_39;
    QLineEdit *endeffMass_3;
    QLabel *label_38;
    QLabel *label_41;
    QGridLayout *gridLayout_8;
    QLineEdit *lowerframe;
    QLineEdit *upperframe;
    QLabel *label_43;
    QLabel *label_44;
    QLabel *label_55;
    QWidget *layoutWidget7;
    QGridLayout *gridLayout_5;
    QLabel *label_34;
    QLabel *label_35;
    QLineEdit *fmax;
    QLabel *label_54;
    QLineEdit *P;
    QLineEdit *fmin;
    QLabel *label_36;
    QLineEdit *d;
    QLineEdit *I;
    QLabel *label_33;
    QLabel *label_32;
    QMenuBar *menuBar;
    QMenu *menuCDPR_Parameters;
    QMenu *menuImport;
    QMenu *menuEdit;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *toolBar;

    void setupUi(QMainWindow *CableDrivenParalleRobot)
    {
        if (CableDrivenParalleRobot->objectName().isEmpty())
            CableDrivenParalleRobot->setObjectName(QStringLiteral("CableDrivenParalleRobot"));
        CableDrivenParalleRobot->setEnabled(true);
        CableDrivenParalleRobot->resize(904, 682);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(CableDrivenParalleRobot->sizePolicy().hasHeightForWidth());
        CableDrivenParalleRobot->setSizePolicy(sizePolicy);
        CableDrivenParalleRobot->setMinimumSize(QSize(904, 682));
        CableDrivenParalleRobot->setMaximumSize(QSize(904, 682));
        actionNew_Window = new QAction(CableDrivenParalleRobot);
        actionNew_Window->setObjectName(QStringLiteral("actionNew_Window"));
        actionClose = new QAction(CableDrivenParalleRobot);
        actionClose->setObjectName(QStringLiteral("actionClose"));
        actionFile = new QAction(CableDrivenParalleRobot);
        actionFile->setObjectName(QStringLiteral("actionFile"));
        actionRobot = new QAction(CableDrivenParalleRobot);
        actionRobot->setObjectName(QStringLiteral("actionRobot"));
        actionCopy = new QAction(CableDrivenParalleRobot);
        actionCopy->setObjectName(QStringLiteral("actionCopy"));
        actionPaste = new QAction(CableDrivenParalleRobot);
        actionPaste->setObjectName(QStringLiteral("actionPaste"));
        actionClear = new QAction(CableDrivenParalleRobot);
        actionClear->setObjectName(QStringLiteral("actionClear"));
        centralWidget = new QWidget(CableDrivenParalleRobot);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(730, 536, 91, 21));
        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(710, 600, 131, 21));
        label_16 = new QLabel(centralWidget);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(470, 10, 131, 16));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(20, 30, 359, 318));
        gridLayout_2 = new QGridLayout(layoutWidget);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setSpacing(6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        gridLayout->addWidget(label, 0, 1, 1, 1);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        gridLayout->addWidget(label_2, 0, 2, 1, 1);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        gridLayout->addWidget(label_3, 1, 0, 1, 1);

        anchorpoint1 = new QLineEdit(layoutWidget);
        anchorpoint1->setObjectName(QStringLiteral("anchorpoint1"));

        gridLayout->addWidget(anchorpoint1, 1, 1, 1, 1);

        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 2, 0, 1, 1);

        anchorpoint2 = new QLineEdit(layoutWidget);
        anchorpoint2->setObjectName(QStringLiteral("anchorpoint2"));

        gridLayout->addWidget(anchorpoint2, 2, 1, 1, 1);

        exitpoint2 = new QLineEdit(layoutWidget);
        exitpoint2->setObjectName(QStringLiteral("exitpoint2"));

        gridLayout->addWidget(exitpoint2, 2, 2, 1, 1);

        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        gridLayout->addWidget(label_5, 3, 0, 1, 1);

        anchorpoint3 = new QLineEdit(layoutWidget);
        anchorpoint3->setObjectName(QStringLiteral("anchorpoint3"));

        gridLayout->addWidget(anchorpoint3, 3, 1, 1, 1);

        label_12 = new QLabel(layoutWidget);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout->addWidget(label_12, 0, 0, 1, 1);

        exitpoint3 = new QLineEdit(layoutWidget);
        exitpoint3->setObjectName(QStringLiteral("exitpoint3"));

        gridLayout->addWidget(exitpoint3, 3, 2, 1, 1);

        exitpoint1 = new QLineEdit(layoutWidget);
        exitpoint1->setObjectName(QStringLiteral("exitpoint1"));

        gridLayout->addWidget(exitpoint1, 1, 2, 1, 1);


        gridLayout_2->addLayout(gridLayout, 0, 0, 1, 3);

        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));

        gridLayout_2->addWidget(label_7, 1, 0, 1, 1);

        anchorpoint4 = new QLineEdit(layoutWidget);
        anchorpoint4->setObjectName(QStringLiteral("anchorpoint4"));

        gridLayout_2->addWidget(anchorpoint4, 1, 1, 1, 1);

        exitpoint4 = new QLineEdit(layoutWidget);
        exitpoint4->setObjectName(QStringLiteral("exitpoint4"));

        gridLayout_2->addWidget(exitpoint4, 1, 2, 1, 1);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout_2->addWidget(label_8, 2, 0, 1, 1);

        anchorpoint5 = new QLineEdit(layoutWidget);
        anchorpoint5->setObjectName(QStringLiteral("anchorpoint5"));

        gridLayout_2->addWidget(anchorpoint5, 2, 1, 1, 1);

        exitpoint5 = new QLineEdit(layoutWidget);
        exitpoint5->setObjectName(QStringLiteral("exitpoint5"));

        gridLayout_2->addWidget(exitpoint5, 2, 2, 1, 1);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout_2->addWidget(label_9, 3, 0, 1, 1);

        anchorpoint6 = new QLineEdit(layoutWidget);
        anchorpoint6->setObjectName(QStringLiteral("anchorpoint6"));

        gridLayout_2->addWidget(anchorpoint6, 3, 1, 1, 1);

        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout_2->addWidget(label_10, 4, 0, 1, 1);

        anchorpoint7 = new QLineEdit(layoutWidget);
        anchorpoint7->setObjectName(QStringLiteral("anchorpoint7"));

        gridLayout_2->addWidget(anchorpoint7, 4, 1, 1, 1);

        exitpoint7 = new QLineEdit(layoutWidget);
        exitpoint7->setObjectName(QStringLiteral("exitpoint7"));

        gridLayout_2->addWidget(exitpoint7, 4, 2, 1, 1);

        label_11 = new QLabel(layoutWidget);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_2->addWidget(label_11, 5, 0, 1, 1);

        exitpoint8 = new QLineEdit(layoutWidget);
        exitpoint8->setObjectName(QStringLiteral("exitpoint8"));

        gridLayout_2->addWidget(exitpoint8, 5, 2, 1, 1);

        anchorpoint8 = new QLineEdit(layoutWidget);
        anchorpoint8->setObjectName(QStringLiteral("anchorpoint8"));

        gridLayout_2->addWidget(anchorpoint8, 5, 1, 1, 1);

        exitpoint6 = new QLineEdit(layoutWidget);
        exitpoint6->setObjectName(QStringLiteral("exitpoint6"));

        gridLayout_2->addWidget(exitpoint6, 3, 2, 1, 1);

        layoutWidget1 = new QWidget(centralWidget);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(470, 30, 189, 29));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_17 = new QLabel(layoutWidget1);
        label_17->setObjectName(QStringLiteral("label_17"));

        horizontalLayout_3->addWidget(label_17);

        CableMass = new QLineEdit(layoutWidget1);
        CableMass->setObjectName(QStringLiteral("CableMass"));

        horizontalLayout_3->addWidget(CableMass);

        layoutWidget2 = new QWidget(centralWidget);
        layoutWidget2->setObjectName(QStringLiteral("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(470, 60, 200, 29));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        label_18 = new QLabel(layoutWidget2);
        label_18->setObjectName(QStringLiteral("label_18"));

        horizontalLayout_4->addWidget(label_18);

        CableRadius = new QLineEdit(layoutWidget2);
        CableRadius->setObjectName(QStringLiteral("CableRadius"));

        horizontalLayout_4->addWidget(CableRadius);

        pushButton_3 = new QPushButton(centralWidget);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));
        pushButton_3->setGeometry(QRect(710, 570, 131, 21));
        label_19 = new QLabel(centralWidget);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(30, 470, 121, 17));
        label_29 = new QLabel(centralWidget);
        label_29->setObjectName(QStringLiteral("label_29"));
        label_29->setGeometry(QRect(60, 0, 191, 17));
        CableMass_3 = new QLineEdit(centralWidget);
        CableMass_3->setObjectName(QStringLiteral("CableMass_3"));
        CableMass_3->setGeometry(QRect(575, 756, 81, 27));
        layoutWidget3 = new QWidget(centralWidget);
        layoutWidget3->setObjectName(QStringLiteral("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(10, 490, 332, 118));
        gridLayout_3 = new QGridLayout(layoutWidget3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        label_24 = new QLabel(layoutWidget3);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_3->addWidget(label_24, 1, 2, 1, 1);

        Inertialyy = new QLineEdit(layoutWidget3);
        Inertialyy->setObjectName(QStringLiteral("Inertialyy"));

        gridLayout_3->addWidget(Inertialyy, 1, 3, 1, 1);

        label_26 = new QLabel(layoutWidget3);
        label_26->setObjectName(QStringLiteral("label_26"));

        gridLayout_3->addWidget(label_26, 2, 0, 1, 1);

        Inertialyz = new QLineEdit(layoutWidget3);
        Inertialyz->setObjectName(QStringLiteral("Inertialyz"));

        gridLayout_3->addWidget(Inertialyz, 1, 5, 1, 1);

        label_27 = new QLabel(layoutWidget3);
        label_27->setObjectName(QStringLiteral("label_27"));

        gridLayout_3->addWidget(label_27, 2, 2, 1, 1);

        Inertialyz_2 = new QLineEdit(layoutWidget3);
        Inertialyz_2->setObjectName(QStringLiteral("Inertialyz_2"));

        gridLayout_3->addWidget(Inertialyz_2, 2, 3, 1, 1);

        label_25 = new QLabel(layoutWidget3);
        label_25->setObjectName(QStringLiteral("label_25"));

        gridLayout_3->addWidget(label_25, 2, 4, 1, 1);

        Inertialxy = new QLineEdit(layoutWidget3);
        Inertialxy->setObjectName(QStringLiteral("Inertialxy"));

        gridLayout_3->addWidget(Inertialxy, 0, 3, 1, 1);

        label_6 = new QLabel(layoutWidget3);
        label_6->setObjectName(QStringLiteral("label_6"));

        gridLayout_3->addWidget(label_6, 0, 0, 1, 1);

        Inertialxx = new QLineEdit(layoutWidget3);
        Inertialxx->setObjectName(QStringLiteral("Inertialxx"));

        gridLayout_3->addWidget(Inertialxx, 0, 1, 1, 1);

        label_20 = new QLabel(layoutWidget3);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_3->addWidget(label_20, 0, 2, 1, 1);

        Inertialxz = new QLineEdit(layoutWidget3);
        Inertialxz->setObjectName(QStringLiteral("Inertialxz"));

        gridLayout_3->addWidget(Inertialxz, 0, 5, 1, 1);

        Inertialxy_2 = new QLineEdit(layoutWidget3);
        Inertialxy_2->setObjectName(QStringLiteral("Inertialxy_2"));

        gridLayout_3->addWidget(Inertialxy_2, 1, 1, 1, 1);

        label_21 = new QLabel(layoutWidget3);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_3->addWidget(label_21, 0, 4, 1, 1);

        label_22 = new QLabel(layoutWidget3);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_3->addWidget(label_22, 1, 4, 1, 1);

        Inertialxz_2 = new QLineEdit(layoutWidget3);
        Inertialxz_2->setObjectName(QStringLiteral("Inertialxz_2"));

        gridLayout_3->addWidget(Inertialxz_2, 2, 1, 1, 1);

        label_23 = new QLabel(layoutWidget3);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_3->addWidget(label_23, 1, 0, 1, 1);

        Inertialzz = new QLineEdit(layoutWidget3);
        Inertialzz->setObjectName(QStringLiteral("Inertialzz"));

        gridLayout_3->addWidget(Inertialzz, 2, 5, 1, 1);

        layoutWidget4 = new QWidget(centralWidget);
        layoutWidget4->setObjectName(QStringLiteral("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(30, 370, 306, 89));
        gridLayout_4 = new QGridLayout(layoutWidget4);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        gridLayout_4->setContentsMargins(0, 0, 0, 0);
        label_28 = new QLabel(layoutWidget4);
        label_28->setObjectName(QStringLiteral("label_28"));

        gridLayout_4->addWidget(label_28, 2, 1, 1, 1);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label_13 = new QLabel(layoutWidget4);
        label_13->setObjectName(QStringLiteral("label_13"));

        horizontalLayout->addWidget(label_13);

        endeffMass = new QLineEdit(layoutWidget4);
        endeffMass->setObjectName(QStringLiteral("endeffMass"));

        horizontalLayout->addWidget(endeffMass);


        gridLayout_4->addLayout(horizontalLayout, 1, 0, 1, 2);

        endEffShape_2 = new QComboBox(layoutWidget4);
        endEffShape_2->setObjectName(QStringLiteral("endEffShape_2"));

        gridLayout_4->addWidget(endEffShape_2, 2, 2, 1, 1);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_14 = new QLabel(layoutWidget4);
        label_14->setObjectName(QStringLiteral("label_14"));

        horizontalLayout_2->addWidget(label_14);

        endEffShape = new QComboBox(layoutWidget4);
        endEffShape->setObjectName(QStringLiteral("endEffShape"));

        horizontalLayout_2->addWidget(endEffShape);


        gridLayout_4->addLayout(horizontalLayout_2, 2, 0, 1, 1);

        label_15 = new QLabel(layoutWidget4);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_4->addWidget(label_15, 0, 0, 1, 1);

        layoutWidget5 = new QWidget(centralWidget);
        layoutWidget5->setObjectName(QStringLiteral("layoutWidget5"));
        layoutWidget5->setGeometry(QRect(350, 370, 252, 85));
        gridLayout_6 = new QGridLayout(layoutWidget5);
        gridLayout_6->setSpacing(6);
        gridLayout_6->setContentsMargins(11, 11, 11, 11);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        gridLayout_6->setContentsMargins(0, 0, 0, 0);
        label_31 = new QLabel(layoutWidget5);
        label_31->setObjectName(QStringLiteral("label_31"));

        gridLayout_6->addWidget(label_31, 0, 0, 1, 2);

        label_37 = new QLabel(layoutWidget5);
        label_37->setObjectName(QStringLiteral("label_37"));

        gridLayout_6->addWidget(label_37, 1, 0, 1, 1);

        label_30 = new QLabel(layoutWidget5);
        label_30->setObjectName(QStringLiteral("label_30"));

        gridLayout_6->addWidget(label_30, 2, 0, 1, 1);

        endeffRPY = new QLineEdit(layoutWidget5);
        endeffRPY->setObjectName(QStringLiteral("endeffRPY"));

        gridLayout_6->addWidget(endeffRPY, 2, 1, 1, 1);

        endeffPosition = new QLineEdit(layoutWidget5);
        endeffPosition->setObjectName(QStringLiteral("endeffPosition"));

        gridLayout_6->addWidget(endeffPosition, 1, 1, 1, 1);

        layoutWidget6 = new QWidget(centralWidget);
        layoutWidget6->setObjectName(QStringLiteral("layoutWidget6"));
        layoutWidget6->setGeometry(QRect(420, 140, 332, 159));
        gridLayout_9 = new QGridLayout(layoutWidget6);
        gridLayout_9->setSpacing(6);
        gridLayout_9->setContentsMargins(11, 11, 11, 11);
        gridLayout_9->setObjectName(QStringLiteral("gridLayout_9"));
        gridLayout_9->setContentsMargins(0, 0, 0, 0);
        gridLayout_7 = new QGridLayout();
        gridLayout_7->setSpacing(6);
        gridLayout_7->setObjectName(QStringLiteral("gridLayout_7"));
        endEffShape_5 = new QComboBox(layoutWidget6);
        endEffShape_5->setObjectName(QStringLiteral("endEffShape_5"));

        gridLayout_7->addWidget(endEffShape_5, 2, 2, 1, 1);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        label_40 = new QLabel(layoutWidget6);
        label_40->setObjectName(QStringLiteral("label_40"));

        horizontalLayout_8->addWidget(label_40);

        endEffShape_6 = new QComboBox(layoutWidget6);
        endEffShape_6->setObjectName(QStringLiteral("endEffShape_6"));

        horizontalLayout_8->addWidget(endEffShape_6);


        gridLayout_7->addLayout(horizontalLayout_8, 2, 0, 1, 1);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        label_39 = new QLabel(layoutWidget6);
        label_39->setObjectName(QStringLiteral("label_39"));

        horizontalLayout_7->addWidget(label_39);

        endeffMass_3 = new QLineEdit(layoutWidget6);
        endeffMass_3->setObjectName(QStringLiteral("endeffMass_3"));

        horizontalLayout_7->addWidget(endeffMass_3);


        gridLayout_7->addLayout(horizontalLayout_7, 1, 0, 1, 2);

        label_38 = new QLabel(layoutWidget6);
        label_38->setObjectName(QStringLiteral("label_38"));

        gridLayout_7->addWidget(label_38, 2, 1, 1, 1);

        label_41 = new QLabel(layoutWidget6);
        label_41->setObjectName(QStringLiteral("label_41"));

        gridLayout_7->addWidget(label_41, 0, 0, 1, 2);


        gridLayout_9->addLayout(gridLayout_7, 0, 0, 1, 1);

        gridLayout_8 = new QGridLayout();
        gridLayout_8->setSpacing(6);
        gridLayout_8->setObjectName(QStringLiteral("gridLayout_8"));
        lowerframe = new QLineEdit(layoutWidget6);
        lowerframe->setObjectName(QStringLiteral("lowerframe"));

        gridLayout_8->addWidget(lowerframe, 1, 1, 1, 1);

        upperframe = new QLineEdit(layoutWidget6);
        upperframe->setObjectName(QStringLiteral("upperframe"));

        gridLayout_8->addWidget(upperframe, 2, 1, 1, 1);

        label_43 = new QLabel(layoutWidget6);
        label_43->setObjectName(QStringLiteral("label_43"));

        gridLayout_8->addWidget(label_43, 1, 0, 1, 1);

        label_44 = new QLabel(layoutWidget6);
        label_44->setObjectName(QStringLiteral("label_44"));

        gridLayout_8->addWidget(label_44, 2, 0, 1, 1);


        gridLayout_9->addLayout(gridLayout_8, 1, 0, 1, 1);

        label_55 = new QLabel(centralWidget);
        label_55->setObjectName(QStringLiteral("label_55"));
        label_55->setGeometry(QRect(850, 1060, 41, 27));
        layoutWidget7 = new QWidget(centralWidget);
        layoutWidget7->setObjectName(QStringLiteral("layoutWidget7"));
        layoutWidget7->setGeometry(QRect(370, 480, 311, 91));
        gridLayout_5 = new QGridLayout(layoutWidget7);
        gridLayout_5->setSpacing(6);
        gridLayout_5->setContentsMargins(11, 11, 11, 11);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        gridLayout_5->setContentsMargins(0, 0, 0, 0);
        label_34 = new QLabel(layoutWidget7);
        label_34->setObjectName(QStringLiteral("label_34"));

        gridLayout_5->addWidget(label_34, 1, 1, 1, 1);

        label_35 = new QLabel(layoutWidget7);
        label_35->setObjectName(QStringLiteral("label_35"));

        gridLayout_5->addWidget(label_35, 1, 5, 1, 1);

        fmax = new QLineEdit(layoutWidget7);
        fmax->setObjectName(QStringLiteral("fmax"));

        gridLayout_5->addWidget(fmax, 1, 2, 1, 3);

        label_54 = new QLabel(layoutWidget7);
        label_54->setObjectName(QStringLiteral("label_54"));

        gridLayout_5->addWidget(label_54, 2, 3, 1, 1);

        P = new QLineEdit(layoutWidget7);
        P->setObjectName(QStringLiteral("P"));

        gridLayout_5->addWidget(P, 2, 1, 1, 2);

        fmin = new QLineEdit(layoutWidget7);
        fmin->setObjectName(QStringLiteral("fmin"));

        gridLayout_5->addWidget(fmin, 1, 6, 1, 2);

        label_36 = new QLabel(layoutWidget7);
        label_36->setObjectName(QStringLiteral("label_36"));

        gridLayout_5->addWidget(label_36, 2, 0, 1, 1);

        d = new QLineEdit(layoutWidget7);
        d->setObjectName(QStringLiteral("d"));

        gridLayout_5->addWidget(d, 2, 7, 1, 1);

        I = new QLineEdit(layoutWidget7);
        I->setObjectName(QStringLiteral("I"));

        gridLayout_5->addWidget(I, 2, 4, 1, 2);

        label_33 = new QLabel(layoutWidget7);
        label_33->setObjectName(QStringLiteral("label_33"));

        gridLayout_5->addWidget(label_33, 2, 6, 1, 1);

        label_32 = new QLabel(layoutWidget7);
        label_32->setObjectName(QStringLiteral("label_32"));

        gridLayout_5->addWidget(label_32, 0, 2, 1, 6);

        CableDrivenParalleRobot->setCentralWidget(centralWidget);
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        pushButton->raise();
        pushButton_2->raise();
        label_16->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        pushButton_3->raise();
        label_19->raise();
        label_29->raise();
        CableMass_3->raise();
        label_55->raise();
        menuBar = new QMenuBar(CableDrivenParalleRobot);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 904, 25));
        menuCDPR_Parameters = new QMenu(menuBar);
        menuCDPR_Parameters->setObjectName(QStringLiteral("menuCDPR_Parameters"));
        menuImport = new QMenu(menuCDPR_Parameters);
        menuImport->setObjectName(QStringLiteral("menuImport"));
        menuEdit = new QMenu(menuBar);
        menuEdit->setObjectName(QStringLiteral("menuEdit"));
        CableDrivenParalleRobot->setMenuBar(menuBar);
        mainToolBar = new QToolBar(CableDrivenParalleRobot);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        CableDrivenParalleRobot->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(CableDrivenParalleRobot);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        CableDrivenParalleRobot->setStatusBar(statusBar);
        toolBar = new QToolBar(CableDrivenParalleRobot);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        CableDrivenParalleRobot->addToolBar(Qt::TopToolBarArea, toolBar);

        menuBar->addAction(menuCDPR_Parameters->menuAction());
        menuBar->addAction(menuEdit->menuAction());
        menuCDPR_Parameters->addAction(actionNew_Window);
        menuCDPR_Parameters->addAction(actionClose);
        menuCDPR_Parameters->addAction(menuImport->menuAction());
        menuImport->addAction(actionFile);
        menuImport->addAction(actionRobot);
        menuEdit->addAction(actionCopy);
        menuEdit->addAction(actionPaste);
        menuEdit->addAction(actionClear);

        retranslateUi(CableDrivenParalleRobot);
        QObject::connect(pushButton, SIGNAL(clicked()), CableDrivenParalleRobot, SLOT(close()));

        QMetaObject::connectSlotsByName(CableDrivenParalleRobot);
    } // setupUi

    void retranslateUi(QMainWindow *CableDrivenParalleRobot)
    {
        CableDrivenParalleRobot->setWindowTitle(QApplication::translate("CableDrivenParalleRobot", "CableDrivenParalleRobot", 0));
        actionNew_Window->setText(QApplication::translate("CableDrivenParalleRobot", "Open", 0));
        actionClose->setText(QApplication::translate("CableDrivenParalleRobot", "Close", 0));
        actionFile->setText(QApplication::translate("CableDrivenParalleRobot", "File", 0));
        actionRobot->setText(QApplication::translate("CableDrivenParalleRobot", "Robot", 0));
        actionCopy->setText(QApplication::translate("CableDrivenParalleRobot", "Copy", 0));
        actionPaste->setText(QApplication::translate("CableDrivenParalleRobot", "Paste", 0));
        actionClear->setText(QApplication::translate("CableDrivenParalleRobot", "Clear", 0));
        pushButton->setText(QApplication::translate("CableDrivenParalleRobot", "Close", 0));
        pushButton_2->setText(QApplication::translate("CableDrivenParalleRobot", "Save Parameters", 0));
        label_16->setText(QApplication::translate("CableDrivenParalleRobot", "Cable Paramters", 0));
        label->setText(QApplication::translate("CableDrivenParalleRobot", "Anchor Points", 0));
        label_2->setText(QApplication::translate("CableDrivenParalleRobot", "Exit  Points", 0));
        label_3->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 1", 0));
        label_4->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 2", 0));
        label_5->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 3", 0));
        label_12->setText(QApplication::translate("CableDrivenParalleRobot", "Cable", 0));
        label_7->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 4", 0));
        label_8->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 5", 0));
        label_9->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 6", 0));
        label_10->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 7", 0));
        label_11->setText(QApplication::translate("CableDrivenParalleRobot", "Cable 8", 0));
        label_17->setText(QApplication::translate("CableDrivenParalleRobot", "Mass", 0));
        label_18->setText(QApplication::translate("CableDrivenParalleRobot", "Radius", 0));
        pushButton_3->setText(QApplication::translate("CableDrivenParalleRobot", "Load Parameters", 0));
        label_19->setText(QApplication::translate("CableDrivenParalleRobot", "Inertial Paramters", 0));
        label_29->setText(QApplication::translate("CableDrivenParalleRobot", "Enter Co-ordinates as  X,Y,Z", 0));
        label_24->setText(QApplication::translate("CableDrivenParalleRobot", "Iyy", 0));
        label_26->setText(QApplication::translate("CableDrivenParalleRobot", "Ixz", 0));
        label_27->setText(QApplication::translate("CableDrivenParalleRobot", "Iyz", 0));
        label_25->setText(QApplication::translate("CableDrivenParalleRobot", "Izz", 0));
        label_6->setText(QApplication::translate("CableDrivenParalleRobot", "Ixx", 0));
        label_20->setText(QApplication::translate("CableDrivenParalleRobot", "Ixy", 0));
        label_21->setText(QApplication::translate("CableDrivenParalleRobot", "Ixz", 0));
        label_22->setText(QApplication::translate("CableDrivenParalleRobot", "Iyz", 0));
        label_23->setText(QApplication::translate("CableDrivenParalleRobot", "Ixy", 0));
        label_28->setText(QApplication::translate("CableDrivenParalleRobot", "Color", 0));
        label_13->setText(QApplication::translate("CableDrivenParalleRobot", "Mass", 0));
        endEffShape_2->clear();
        endEffShape_2->insertItems(0, QStringList()
         << QApplication::translate("CableDrivenParalleRobot", "Black", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Blue", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Grey", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Yellow", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Green", 0)
         << QApplication::translate("CableDrivenParalleRobot", "White", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Orange", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Red", 0)
        );
        label_14->setText(QApplication::translate("CableDrivenParalleRobot", "Shape", 0));
        endEffShape->clear();
        endEffShape->insertItems(0, QStringList()
         << QApplication::translate("CableDrivenParalleRobot", "Sphere", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Box", 0)
        );
        label_15->setText(QApplication::translate("CableDrivenParalleRobot", "End-Effector Paramters", 0));
        label_31->setText(QApplication::translate("CableDrivenParalleRobot", "Initial End-Effector Pose", 0));
        label_37->setText(QApplication::translate("CableDrivenParalleRobot", "Position X,Y,Z", 0));
        label_30->setText(QApplication::translate("CableDrivenParalleRobot", "Orientation R,P,Y", 0));
        endEffShape_5->clear();
        endEffShape_5->insertItems(0, QStringList()
         << QApplication::translate("CableDrivenParalleRobot", "Black", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Blue", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Grey", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Yellow", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Green", 0)
         << QApplication::translate("CableDrivenParalleRobot", "White", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Orange", 0)
         << QApplication::translate("CableDrivenParalleRobot", "Red", 0)
        );
        label_40->setText(QApplication::translate("CableDrivenParalleRobot", "Shape", 0));
        endEffShape_6->clear();
        endEffShape_6->insertItems(0, QStringList()
         << QApplication::translate("CableDrivenParalleRobot", "Box", 0)
        );
        label_39->setText(QApplication::translate("CableDrivenParalleRobot", "Mass", 0));
        label_38->setText(QApplication::translate("CableDrivenParalleRobot", "Color", 0));
        label_41->setText(QApplication::translate("CableDrivenParalleRobot", "Bounding Frame Paramters", 0));
        label_43->setText(QApplication::translate("CableDrivenParalleRobot", "Lowest Edge co-ordinate", 0));
        label_44->setText(QApplication::translate("CableDrivenParalleRobot", "Uppermost Edge co-ordinate", 0));
        label_55->setText(QApplication::translate("CableDrivenParalleRobot", "Fmin", 0));
        label_34->setText(QApplication::translate("CableDrivenParalleRobot", "Fmax", 0));
        label_35->setText(QApplication::translate("CableDrivenParalleRobot", "Fmin", 0));
        label_54->setText(QApplication::translate("CableDrivenParalleRobot", "I", 0));
        label_36->setText(QApplication::translate("CableDrivenParalleRobot", "P", 0));
        label_33->setText(QApplication::translate("CableDrivenParalleRobot", "D", 0));
        label_32->setText(QApplication::translate("CableDrivenParalleRobot", "Control Paramters", 0));
        menuCDPR_Parameters->setTitle(QApplication::translate("CableDrivenParalleRobot", "File", 0));
        menuImport->setTitle(QApplication::translate("CableDrivenParalleRobot", "Import", 0));
        menuEdit->setTitle(QApplication::translate("CableDrivenParalleRobot", "Edit", 0));
        toolBar->setWindowTitle(QApplication::translate("CableDrivenParalleRobot", "toolBar", 0));
    } // retranslateUi

};

namespace Ui {
    class CableDrivenParalleRobot: public Ui_CableDrivenParalleRobot {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CABLEDRIVENPARALLEROBOT_H
