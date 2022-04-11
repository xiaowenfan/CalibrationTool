#include "QtWidgetsApplication1.h"
#include <QDebug>
#include <qcompleter.h>
#include <qstringlist.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
//测试

#define PI 3.1415926
//#define image_count 22
//#define w_sq 1.014//24



using namespace Eigen;
using namespace std;
using namespace cv;


Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
Mat distCoeffs = Mat::zeros(8, 1, CV_64F);
vector<vector<Vec2f>> img_points;
vector<Mat> rvecs, tvecs;



QtWidgetsApplication1::QtWidgetsApplication1(QWidget *parent)
    : QWidget(parent)
{
    ui.setupUi(this);


    connect(ui.pushButton,&QPushButton::pressed, this, &QtWidgetsApplication1::calibrate, Qt::AutoConnection);
    setWindowTitle("Calibration Tool");

    connect(ui.pushButton_2, &QPushButton::pressed, this, &QtWidgetsApplication1::Openfile, Qt::AutoConnection);

    //Mat img = imread("F:/Shaw/projects/VS_Projects/test01/QtWidgetsApplication1/chessboard.bmp");//file path
   
    //QImage Qtemp = QImage((const unsigned char*)(img.data), img.cols, img.rows, img.step, QImage::Format_RGB888);
    //QLabel* label = new QLabel(this);
    //label->move(50, 50);
    //label->setPixmap(QPixmap::fromImage(Qtemp));
    //label->resize(Qtemp.size());

}


void  QtWidgetsApplication1::Openfile() {
    QString OpenFilePath, strs;
    QImage image;
    //打开文件夹中的图片文件
    OpenFileList = QFileDialog::getOpenFileNames(this,
        "Please choose an image file",
        "",
        "Image Files(*.jpg *.png *.bmp *.pgm *.pbm);;All files(*.*)");
    //if (OpenFile != "")
    //{
    //    if (image.load(OpenFile))
    //    {
    //        ui.label->setPixmap(QPixmap::fromImage(image));
    //    }
    //}

    //显示所示图片的路径
    if (OpenFileList.size() != 0){
        for (int i = 0; i < OpenFileList.size(); i++) {
            QFileInfo OpenFileInfo;
            OpenFileInfo = QFileInfo(OpenFileList[i]);
            OpenFilePath = OpenFileInfo.filePath();
            strs.append(OpenFilePath);
            strs += ";";
        }
    ui.lineEdit->setText(strs);
    }
}


void QtWidgetsApplication1::compute() {

    QString wid = ui.lineEdit_2->text();
    QString hei = ui.lineEdit_3->text();
    QString dist = ui.lineEdit_4->text();
    disteach = dist.toFloat();
    numCornersHor = wid.toInt();
    numCornersVer = hei.toInt();

    file_num = OpenFileList.size();
    //int numCornersHor = 15;
    //int numCornersVer = 8;
    point_num = numCornersHor * numCornersVer;
    //float disteach = 24.0;
    //string root = "L:/photos/圆点图片";
    vector<vector<Vec3f>> obj_points;
    //vector<vector<Vec2f>> img_points;


    vector<Vec3f> obj_point;
    
    Size pattern_size = Size(numCornersHor, numCornersVer);
    //////initial the world point /////////////////
    obj_point.clear();
    for (int i = 0; i < numCornersVer; i++)
        for (int j = 0; j < numCornersHor; j++)
            obj_point.push_back(Vec3f(float(j * disteach / 1.0), float(i * disteach / 1.0), 0.0f));
    for (int i = 0; i < file_num; i++)
        obj_points.push_back(obj_point);


    //////initial the image point /////////////////
    Mat phase_gray;
    vector<Vec2f> corners;
    for (int i = 0; i < file_num; i++) {
        string f_phase = OpenFileList[i].toStdString();
        //string f_phase = root + "/" + to_string(i) + ".bmp";

        ui.label_29->setText("Reading the " + QString::number(i + 1) + "'s picture...");

        Mat srcImage = imread(f_phase);
        cvtColor(srcImage, phase_gray, COLOR_BGR2GRAY);
        int rows = srcImage.rows;
        int cols = srcImage.cols;


        SimpleBlobDetector::Params params;
        params.minThreshold = 10;
        params.maxThreshold = 200;
        params.filterByArea = true;
        params.minArea = 100;
        params.maxArea = 10e6;
        params.minDistBetweenBlobs = 10;
        params.blobColor = 255;
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        bool found = findCirclesGrid(phase_gray, pattern_size, corners, CALIB_CB_SYMMETRIC_GRID || CALIB_CB_ASYMMETRIC_GRID, detector);
        img_points.push_back(corners);
        
        drawChessboardCorners(srcImage, pattern_size, corners, found);
        //////////////image show/////////////////////
        //QImage Qtemp(OpenFileList[i]);
        //Mat img = imread(f_phase);
        QImage Qtemp = QImage((const unsigned char*)(srcImage.data), srcImage.cols, srcImage.rows, srcImage.step, QImage::Format_RGB888);//Format_RGB888
        QImage res = Qtemp.scaled(ui.label_26->size(),
            Qt::KeepAspectRatio,
            Qt::SmoothTransformation);
 
        ui.label_26->setPixmap(QPixmap::fromImage(res));
        //ui.label_26->setScaledContents(true);
        waitKey(200);
        /////////////////////////////////////////////
        
    }
    /////////////////标定////////////////////////////
    ui.label_29->setText("computing...");
    double rms = cv::calibrateCamera(obj_points, img_points, pattern_size, cameraMatrix, distCoeffs, rvecs, tvecs);
    
    //qDebug() << "rms = " << rms;
    //qDebug() << "Camera intrinsic : \n" << cameraMatrix.at<double>(0,0);
    //qDebug() << "distCoeffs : \n" << distCoeffs << endl;
}


void  QtWidgetsApplication1::calibrate() {

    //运算
    compute();

    ui.label_29->setText("Done!!!");

    ////////if checkBox is true,write the corner_file(1_u.txt/1_v.txt)
    QString change_file_path;
    change_file_path = OpenFileList[1].left(OpenFileList[1].size() - 5);
    if (ui.checkBox->isChecked())
        WritetoFile(change_file_path);
   


    fx = cameraMatrix.at<double>(0, 0);
    fy = cameraMatrix.at<double>(1, 1);
    u0 = cameraMatrix.at<double>(0, 2);
    v0 = cameraMatrix.at<double>(1, 2);

    k1 = distCoeffs.at<double>(0, 0);
    k2 = distCoeffs.at<double>(1, 0);
    p1 = distCoeffs.at<double>(2, 0);
    p2 = distCoeffs.at<double>(3, 0);
    k3 = distCoeffs.at<double>(4, 0);
    

    ui.label->setText(QString::number(fx));
    ui.label_9->setText(QString::number(fy));
    ui.label_7->setText(QString::number(u0));
    ui.label_8->setText(QString::number(v0));
    ui.label_10->setText(QString::number(k1));
    ui.label_15->setText(QString::number(k2));
    ui.label_11->setText(QString::number(k3));
    ui.label_17->setText(QString::number(p1));
    ui.label_19->setText(QString::number(p2));
    ui.label_21->setText(QString::number(0.00));
    
}



void QtWidgetsApplication1::WritetoFile(QString& Qroot) {//root == F:/Shaw/circle_photo/(pictures)
    string root = Qroot.toStdString();
    //写入特征点检测的像素坐标
    //for (int i = 0; i < file_num; i++) {//filenum == the number of picture
    //    string path_u = root + to_string(i + 1) + "_u.txt";
    //    string path_v = root + to_string(i + 1) + "_v.txt";
    //    ofstream file_u(path_u, ios::app);
    //    ofstream file_v(path_v, ios::app);
    //    if (!file_u || !file_v) {
    //        std::cout << "Unable to open otfile";
    //        exit(1);
    //    }
    //    int count = 1;//use to \n in file 
    //    for (int p = 0; p < point_num; p++) {//point_num == the number of points each picture
    //        file_u << fixed << setprecision(6) << (float)img_points[i][p][0] << " ";
    //        file_v << fixed << setprecision(6) << (float)img_points[i][p][1] << " ";
    //        if (count % numCornersHor == 0) {
    //            file_v << endl;
    //            file_u << endl;
    //        }
    //        count++;
    //    }
    //}
    
    //写入标定的参数
    string path_parameter = root + "外参数.txt";
    ofstream file_parameter(path_parameter, ios::app);
    if (!file_parameter) {
        std::cout << "Unable to open otfile";
        exit(1);
    }
    //写入旋转向量和平移向量
    for (int i = 0; i < file_num; i++) {
        for(int j = 0; j< 3;j++)
            file_parameter << fixed << setprecision(17)<< rvecs[i].at<double>(0,j) << endl;
        for(int j = 0;j < 3;j++)
            file_parameter << fixed << setprecision(17) << tvecs[i].at<double>(0,j) << endl;
    }


}


void QtWidgetsApplication1::telecalibrate() {


    QString wid = ui.lineEdit_2->text();
    QString hei = ui.lineEdit_3->text();
    QString dist = ui.lineEdit_4->text();
    disteach = dist.toFloat();
    numCornersHor = wid.toInt();
    numCornersVer = hei.toInt();

    file_num = OpenFileList.size();
    point_num = numCornersHor * numCornersVer;
    


    int w, h;
    w = 12;//cols
    h = 9;//rows
    int num_point = w * h;
    Size pattern_size = Size(w, h);
    //Point corners[w * h];
    Mat srcImage, grayImage;
    string filename, path, imname;
    double m;//==a wei zhi shu
    //path = root;

    //////////////Generate world coordinates////////////////
    vector<Point2f> object_points;
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            Point2f realPoint;
            realPoint.x = j * disteach;
            realPoint.y = i * disteach;
            object_points.push_back(realPoint);
        }
    }

    ////////////////Generate [Xw Yw 1 0 0 0] [0 0 0 Xw Yw 1] matrix///////////////////////////
    Matrix<double, Dynamic, Dynamic> M;
    M.setZero(216, 216);                          //need change per kind of file  2*point_num 
    for (int p = 0, k = 0; k < 2 * num_point; k += 2) {
        M(k, 0) = object_points[p].x;
        M(k, 1) = object_points[p].y;
        M(k, 2) = 1;
        M(k + 1, 3) = object_points[p].x;
        M(k + 1, 4) = object_points[p].y;
        M(k + 1, 5) = 1;
        p++;
    }

    /////////////initial guess for u0,v0////////////////////
    //Size pic_size = srcImage.size();
    int u0;
    int v0;
    
    vector<Matrix<double, 3, 3>> homographies(file_num, Matrix<double, 3, 3>(3, 3));
    vector<Matrix<double, 3, 3>> k_mats(file_num, Matrix<double, 3, 3>(3, 3));
    vector<Matrix<double, Dynamic, 1>> corner(file_num, Matrix<double, Dynamic, 1>(2 * point_num, 1));
    //cout << "M_ \n"<< M << endl;

    /// <summary>
    /// ///////////////////////////////////////从这里开始处理每张图/////////////////////////////////////
    /// </summary>
    for (int i = 0; i < file_num; i++) {
        string filename = OpenFileList[i].toStdString();
        srcImage = imread(filename);
        imname = "img" + to_string(i);
        u0 = srcImage.cols / 2;
        v0 = srcImage.rows / 2;
        //         cout << "u0 = " << u0 << "\n" << "v0 = " << v0 <<endl;

        Mat corners;
        Mat gray;
        cvtColor(srcImage, gray, COLOR_BGR2GRAY);
        //////////////inverse the color when the pattern is chessboard///////////////
        int rows = srcImage.rows;
        int cols = srcImage.cols;
        //         for(int row = 0;row < rows;row++){          //chessboard need inverse color
        //             for(int col = 0;col < cols;col++)
        //                 gray.at<uchar>(row,col) = 255 - gray.at<uchar>(row,col);
        //         }



            ////////////////////set the params when the pattern is circle//////////////////////
        SimpleBlobDetector::Params params;
        //阈值控制
        params.minThreshold = 10;
        params.maxThreshold = 200;
        //像素面积大小控制
        params.filterByArea = true;
        params.minArea = 100;
        params.maxArea = 10e6;
        params.minDistBetweenBlobs = 10;
        params.blobColor = 255;
        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        bool found = findCirclesGrid(gray, pattern_size, corners, CALIB_CB_SYMMETRIC_GRID, detector);
        //bool found = findChessboardCorners(gray,pattern_size,corners,CALIB_CB_SYMMETRIC_GRID);

        drawChessboardCorners(srcImage, pattern_size, corners, found);

        cout << "image_" << to_string(i) << ": " << found << endl;
        imshow(imname, srcImage);
        waitKey(100);



        for (int j = 0; j < 2 * point_num; j++)
            corner[i - 1](j, 0) = corners.at<float>(0, j);

        //         cout << corners.at<float>(0,0) << "  " << corners.at<float>(0,1) << "  " << corners.at<float>(0,2) << "  " << corners.at<float>(0,3) << endl;
                /////////solve homography matrix each picture///////////////////
        Matrix<double, 216, 1> h = M.colPivHouseholderQr().solve(corner[i - 1]); //colPivHouseholderQr()        ////////////need change the number 216 per kind of file////////////////////////
//         cout<< "h = \n" << h << endl;

        //cout<< "h = \n" << h << endl;
        homographies[i - 1](0, 0) = h(0, 0);
        homographies[i - 1](0, 1) = h(1, 0);
        homographies[i - 1](0, 2) = h(2, 0);
        homographies[i - 1](1, 0) = h(3, 0);
        homographies[i - 1](1, 1) = h(4, 0);
        homographies[i - 1](1, 2) = h(5, 0);
        homographies[i - 1](2, 0) = 0;
        homographies[i - 1](2, 1) = 0;
        homographies[i - 1](2, 2) = 1;
        //cout << "homographies[" << to_string(i-1) << "] = \n" << homographies[i-1] << endl;
//         cout<<"h(1)= "<< h(0,0)<< "    " << "homographies= " << homographies[i-1](0,0) << "\n";
//         cout<<"h(2)= "<< h(1,0)<< "    " << "homographies= " << homographies[i-1](0,1) << "\n";
//         cout<<"h(3)= "<< h(2,0)<< "    " << "homographies= " << homographies[i-1](0,2) << "\n";
//         cout<<"h(4)= "<< h(3,0)<< "    " << "homographies= " << homographies[i-1](1,0) << "\n";
//         cout<<"h(5)= "<< h(4,0)<< "    " << "homographies= " << homographies[i-1](1,1) << "\n";
//         cout<<"h(6)= "<< h(5,0)<< "    " << "homographies= " << homographies[i-1](1,2) << "\n";
//         cout<<"h(80)= "<< h(80,0)<< "    " << "homographies[i](2,0)= " << homographies[i-1](2,0) << "\n";
//         cout<<"h(7)= "<< h(6,0)<< "    " << "homographies[i](2,2)= " << homographies[i-1](2,2) << endl;

        ///////////////solve polynomial///////////////////////
        double c1 = h(0, 0) * h(0, 0) + h(1, 0) * h(1, 0) + h(3, 0) * h(3, 0) + h(4, 0) * h(4, 0);
        double c2 = (h(0, 0) * h(4, 0) - h(1, 0) * h(3, 0)) * (h(0, 0) * h(4, 0) - h(1, 0) * h(3, 0));
        Mat coef = (Mat_<double>(5, 1) << c2, 0, -c1, 0, 1);
        Mat roots;
        solvePoly(coef, roots);
        //cout<<"Roots: channels = " << roots.channels() << ",value = " << roots << ".\n";
        vector<double> coff = { roots.at<double>(0,0),roots.at<double>(1,0),roots.at<double>(2,0),roots.at<double>(3,0) };
        m = *max_element(coff.begin(), coff.end());
        //         cout << "a = " << m << endl;
        //         cout << "h13/a = " << (h(2,0)-u0)/m << endl;
        //         cout << "h23/a = " << (h(5,0)-v0)/m << endl;
        k_mats[i - 1] << m, 0, u0, 0, m, v0, 0, 0, 1;
        //cout << "k_mats : \n" << k_mats[i-1] << endl;
    }


    //////////////////solve alpha and beta////////////////////////////
    Matrix<double, Dynamic, Dynamic> G;
    G.setOnes(Dynamic, Dynamic);
    Matrix<double, Dynamic, 1> W;
    W.setZero(Dynamic, 1);


    for (int k = 0; k < file_num; k++) { //solve intrinsic matrix K_lsp
        Matrix<double, 3, 3> H = homographies[k];
        G(k, 1) = -(H(1, 0) * H(1, 0)) - (H(1, 1) * H(1, 1));
        G(k, 2) = -(H(0, 0) * H(0, 0)) - (H(0, 1) * H(0, 1));
        G(k, 3) = 2 * (H(0, 0) * H(1, 0) + H(0, 1) * H(1, 1));
        W(k, 0) = -((H(0, 0) * H(1, 1) - H(0, 1) * H(1, 0)) * (H(0, 0) * H(1, 1) - H(0, 1) * H(1, 0)));
    }
    Matrix<double, Dynamic, 1> L = G.colPivHouseholderQr().solve(W);

    double alpha = sqrt((L(1, 0) * L(2, 0) - L(3, 0) * L(3, 0)) / L(2, 0));
    double beta = sqrt(L(2, 0));
    double gama = sqrt(L(1, 0) - (L(0, 0) / L(2, 0)));
    cout << "alpha =  " << alpha << "\n" << "beta =  " << beta << endl;
    //cout << "gama = " << gama << endl;

    Matrix<double, 3, 3> K_lsp;
    K_lsp << alpha, 0, u0, 0, beta, v0, 0, 0, 1;


    Vector3d T;
    Mat R, r_vec;


    double even_error_u = 0;
    double even_error_v = 0;




    ///////////////////////////////////////////////又是一个for循环///////////////////////////
    for (int k = 0; k < file_num; k++) {//solve R and T



        Matrix<double, 3, 3> H = homographies[k];
        Matrix<double, 3, 3> K = k_mats[k];

        Matrix<double, 3, 3> E = K.colPivHouseholderQr().solve(H);

        //         cout << "e31 = " << E(2,0) << endl;
        //         cout << "e32 = " << E(2,1) << endl;
        //         cout << "e33 = " << E(2,2) << endl;
        //         cout << "t_x = " << E(0,2) << endl;
        //         cout << "t_y = " << E(1,2) << endl;


        double tx = (H(0, 2) - u0) / m;//tx == E(0,2)
        double ty = (H(1, 2) - v0) / m;//ty == E(1,2)

        double r13 = sqrt(1 - E(0, 0) * E(0, 0) - E(1, 0) * E(1, 0));//according the property of unit vector "x^2 + y^2 + z^2 = 1" to solve r13
        double r23 = sqrt(1 - E(0, 1) * E(0, 1) - E(1, 1) * E(1, 1));
        Vector3d r1(E(0, 0), E(0, 1), r13);
        Vector3d r2(E(1, 0), E(1, 1), r23);
        Vector3d r3 = r1.cross(r2);

        T << tx, ty, 0;
        Matrix3d Rm;
        Rm << r1, r2, r3;//yi lie yi lie de 

        cout << "***********file__" << to_string(k + 1) << "__*************" << endl;
        R = (Mat_<double>(3, 3) << E(0, 0), E(0, 1), r13,
            E(1, 0), E(1, 1), r23,
            r3(0, 0), r3(1, 0), r3(2, 0));
        //         cout << "R * RT = \n" << Rm * Rm.inverse() << endl;

        cout << "R_" << to_string(k) << "_ : \n" << R << endl;
        //         cout << "R[0,0] = " << R.at<double>(0,1) << endl;

        //         Rodrigues(R,r_vec);
        //         cout << "R" << to_string(k) << "_ : \n" << r_vec << endl;
        cout << "T_" << to_string(k) << "_ : \n" << T << endl;
        //         cout << "T[0]= " << T[0] << "    T[1]= " << T[1] << endl;


        //         string error_path_u = path + to_string(k) + "_error_u.txt";
        //         string error_path_v = path + to_string(k) + "_error_v.txt";
        //         ofstream error_u(error_path_u, ios::app);
        //         ofstream error_v(error_path_v, ios::app);
        //         
        //         if(!error_u || !error_v){
        //             cout << "Unable to open otfile";
        //             exit(1); 
        //         }

        vector<double> e_u(point_num);
        vector<double> e_v(point_num);
        double pic_error_u = 0;
        double pic_error_v = 0;
        for (int i = 0, j = 0; j < 2 * point_num; i++, j += 2) {
            double xp = 0;
            double yp = 0;
            //             xp =  (alpha * R.at<double>(0,0) + gama * R.at<double>(1,0)) * object_points[i].x + (alpha * R.at<double>(0,1) + gama * R.at<double>(1,1))* object_points[i].y + alpha * T[0] + gama * T[1] + u0;
            //             yp =  beta * R.at<double>(1,0) * object_points[i].x + beta * R.at<double>(1,1) * object_points[i].y + beta * T[1] + v0;
            xp = alpha * R.at<double>(0, 0) * object_points[i].x + alpha * R.at<double>(0, 1) * object_points[i].y + alpha * T[0] + u0;
            yp = beta * R.at<double>(1, 0) * object_points[i].x + beta * R.at<double>(1, 1) * object_points[i].y + beta * T[1] + v0;
            //             xp =  alpha * (R.at<double>(0,0) * object_points[i].x + R.at<double>(0,1) * object_points[i].y + T[0]);
            //             yp =  beta * (R.at<double>(1,0) * object_points[i].x + R.at<double>(1,1) * object_points[i].y + T[1]);
            e_u[i] = corner[k](j, 0) - xp;
            e_v[i] = corner[k](j + 1, 0) - yp;
            pic_error_u = pic_error_u + abs(corner[k](j, 0) - xp);
            pic_error_v = pic_error_v + abs(corner[k](j + 1, 0) - yp);
        }

        //         cout << "pic_error_u = " << pic_error_u/point_num << endl;
        //         cout << "pic_error_v = " << pic_error_v/point_num << endl;
        //         even_error_u += pic_error_u/point_num;
        //         even_error_v += pic_error_v/point_num;

    }

    destroyAllWindows();
}