#pragma once

#include <QtWidgets/QWidget>
#include "ui_QtWidgetsApplication1.h"
#include <QDebug>
#include <iostream>  
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <QVector>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>


class QtWidgetsApplication1 : public QWidget,public Ui_QtWidgetsApplication1Class
{
    Q_OBJECT
    
public:
    QtWidgetsApplication1(QWidget *parent = Q_NULLPTR);
    void calibrate();
    void Openfile();
    void compute();
    void telecalibrate();
   // void RefineCoordinates(const string& root);
   // void wrap_phase(const string& root);
    void WritetoFile(QString &root);
  //  void Calibrate();
   // void findminvalue(const string& root);
   // void surfacefit(const string& root);
   // void averagephase(const string& root);
   // void new_method(const string& root);
    //void findcircle(const string& root);
    
    
private:
    Ui::QtWidgetsApplication1Class ui;
    QStringList OpenFileList;

    float disteach;
    int numCornersHor;
    int numCornersVer;
    int file_num;
    int point_num;

    double fx;
    double fy;
    double u0;
    double v0;

    double k1;
    double k2;
    double p1;
    double p2;
    double k3;

};
