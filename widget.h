#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QtSerialPort/QSerialPort>// 提供访问串口的功能
#include <QtSerialPort/QSerialPortInfo>// 提供系统中存在的串口信息
#include <QTimer>
#include <QRadioButton>
#include <QString>
/*#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QtOpenGL/QOpenGLTexture>
*/
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <stdio.h>
#include <QElapsedTimer>

QT_BEGIN_NAMESPACE
namespace Ui {
class Widget;
}
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);

    ~Widget();

    void Serial_InitialSetting();

    void SearchSerialPorts();

    bool eventFilter(QObject *obj, QEvent *event);

    void Control_InitialSetting();

    float get_depth_scale(rs2::device dev);


private:

    //void Coordinate_trans(float posnow_z,float posobj_z);

    cv::Mat Get3DR_TransMatrix(const std::vector<cv::Point3f>& srcPoints, const std::vector<cv::Point3f>&  dstPoints)
    {
        double srcSumX = 0.0f;
        double srcSumY = 0.0f;
        double srcSumZ = 0.0f;

        double dstSumX = 0.0f;
        double dstSumY = 0.0f;
        double dstSumZ = 0.0f;

        //至少三组点
        if (srcPoints.size() != dstPoints.size() || srcPoints.size() < 3)
        {
            return cv::Mat();
        }

        int pointsNum = srcPoints.size();
        for (int i = 0; i < pointsNum; ++i)
        {
            srcSumX += srcPoints[i].x;
            srcSumY += srcPoints[i].y;
            srcSumZ += srcPoints[i].z;

            dstSumX += dstPoints[i].x;
            dstSumY += dstPoints[i].y;
            dstSumZ += dstPoints[i].z;
        }

        cv::Point3d centerSrc, centerDst;

        centerSrc.x = double(srcSumX / pointsNum);
        centerSrc.y = double(srcSumY / pointsNum);
        centerSrc.z = double(srcSumZ / pointsNum);

        centerDst.x = double(dstSumX / pointsNum);
        centerDst.y = double(dstSumY / pointsNum);
        centerDst.z = double(dstSumZ / pointsNum);

        //Mat::Mat(int rows, int cols, int type)
        cv::Mat srcMat(3, pointsNum, CV_64FC1);
        cv::Mat dstMat(3, pointsNum, CV_64FC1);
        //---Modify
        for (int i = 0; i < pointsNum; ++i)//N组点
        {
            //三行
            srcMat.at<double>(0, i) = srcPoints[i].x - centerSrc.x;
            srcMat.at<double>(1, i) = srcPoints[i].y - centerSrc.y;
            srcMat.at<double>(2, i) = srcPoints[i].z - centerSrc.z;

            dstMat.at<double>(0, i) = dstPoints[i].x - centerDst.x;
            dstMat.at<double>(1, i) = dstPoints[i].y - centerDst.y;
            dstMat.at<double>(2, i) = dstPoints[i].z - centerDst.z;

        }

        cv::Mat matS = srcMat * dstMat.t();

        cv::Mat matU, matW, matV;
        cv::SVDecomp(matS, matW, matU, matV);

        cv::Mat matTemp = matU * matV;
        double det = cv::determinant(matTemp);//行列式的值

        double datM[] = { 1, 0, 0, 0, 1, 0, 0, 0, det };
        cv::Mat matM(3, 3, CV_64FC1, datM);

        cv::Mat matR = matV.t() * matM * matU.t();

        double* datR = (double*)(matR.data);
        double delta_X = centerDst.x - (centerSrc.x * datR[0] + centerSrc.y * datR[1] + centerSrc.z * datR[2]);
        double delta_Y = centerDst.y - (centerSrc.x * datR[3] + centerSrc.y * datR[4] + centerSrc.z * datR[5]);
        double delta_Z = centerDst.z - (centerSrc.x * datR[6] + centerSrc.y * datR[7] + centerSrc.z * datR[8]);


        //生成RT齐次矩阵(4*4)
        cv::Mat R_T = (cv::Mat_<double>(4, 4) <<
                           matR.at<double>(0, 0), matR.at<double>(0, 1), matR.at<double>(0, 2), delta_X,
                       matR.at<double>(1, 0), matR.at<double>(1, 1), matR.at<double>(1, 2), delta_Y,
                       matR.at<double>(2, 0), matR.at<double>(2, 1), matR.at<double>(2, 2), delta_Z,
                       0, 0, 0, 1
                       );

        return R_T;
    }

    cv::Mat align_Depth2Color(cv::Mat depth,cv::Mat color,rs2::pipeline_profile profile);

    void measure_distance(cv::Mat &color,cv::Mat depth,cv::Size range,rs2::pipeline_profile profile);

    void readData();

    void writeDone(QByteArray byteArray);

    QByteArray serialdata_axial_speedfilter(double speed);

    struct ReturnData time_control(double speed , double distance);

    int digits_compute(double number);

private slots:

    //void on_pushButton_Open_clicked(bool checked);

    //void on_pushButton_Open_clicked();
    void updateDepthImageAndColorBar();

    void on_pushButton_OpenorClose_clicked();

    void on_pushButton_Refresh_clicked();

    //bool writeDone(qint64 data);

    void on_comboBox_State_axial_currentIndexChanged(int index);

    void on_comboBox_State_Radial_currentIndexChanged(int index);

    //void on_pushButton_Axial_clicked();

    // void on_doubleSpinBox_Speed_valueChanged(double speed);

    //void on_doubleSpinBox_Speed_editingFinished();

    void on_pushButton_Move_clicked();

    void on_pushButton_Clear_clicked();

private:
    Ui::Widget *ui;
    QSerialPort *serialPort;
    QTimer *timer,*timer1,*timer2;
    QElapsedTimer E_timer;
};
#endif // WIDGET_H
