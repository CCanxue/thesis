#include "widget.h"
#include "./ui_widget.h"
#include <QtSerialPort/QSerialPort>// 提供访问串口的功能
#include <QtSerialPort/QSerialPortInfo>// 提供系统中存在的串口信息
#include <QMessageBox>
#include <QDebug>
#include <QKeyEvent>
#include <QValidator>
#include <QObject>
#include <QString>
#include <QTextCodec>
#include <QTimer>
#include <stdio.h>
#include <QtMath>
#include <QElapsedTimer>
/*
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QtOpenGL/QOpenGLTexture>
*/
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsPixmapItem>
#include <opencv2/opencv.hpp>
using namespace rs2;
using namespace cv;

struct ReturnData {
    int time;
    int flag;
};

int restrictflag = 0;

rs2::context ctx;
rs2::config cfg;
rs2::pipeline p;
rs2::pipeline_profile profile;
float Camera_Location[3];
Mat RT;


Widget::Widget(QWidget *parent)
    : QWidget(parent),ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowTitle("串口通讯");
    serialPort = new QSerialPort;

    //QStringList Portstring;
    //connect(&serialPort,&QSerialPort::readyRead,this,&Widget)

    // 连接读取数据信号与槽函数
    //connect(serialPort, &QSerialPort::readyRead, this, &Widget::readData);

    // 连接数据发送完成信号与槽函数
    //connect(serialPort, &QSerialPort::bytesWritten, this, &Widget::writeDone);  //
    ui->radioButton_State->installEventFilter(this);
    ui->doubleSpinBox_Axialspeed->installEventFilter(this);
    /*
    ui->lineEdit_Backward->setValidator(new QDoubleValidator(this));
    ui->lineEdit_Forward->setValidator(new QDoubleValidator(this));
    ui->lineEdit_Forward->installEventFilter(this);
    ui->lineEdit_Backward->installEventFilter(this);
    */
    //ui->radioButton_State->setCheckable(false);
    Serial_InitialSetting();
    Control_InitialSetting();
    // 创建源点集
    std::vector<cv::Point3f> srcPoints;
    srcPoints.push_back(cv::Point3f(-0.181183 , 0.245726 , 0.54));
    srcPoints.push_back(cv::Point3f(-0.198311 , 0.252855 , 0.595));
    srcPoints.push_back(cv::Point3f(-0.217553 , 0.255661 , 0.642));
    // ... 添加更多的点 ...

    // 创建目标点集
    std::vector<cv::Point3f> dstPoints;
    dstPoints.push_back(cv::Point3f(0,0,0));
    dstPoints.push_back(cv::Point3f(0,0,0.05));
    dstPoints.push_back(cv::Point3f(0,0,0.10));
    // ... 添加更多的点 ..

    RT = Get3DR_TransMatrix(srcPoints,dstPoints);

    for (int i = 0; i < RT.rows; ++i) {
        for (int j = 0; j < RT.cols; ++j) {
            qDebug() << RT.at<float>(i, j) << " ";
        }
    }

    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);
    profile = p.start(cfg);
    profile.get_device();
    rs2::device device = profile.get_device();
    device.hardware_reset();

    //QTimer::singleShot(0, this, &Widget::updateDepthImageAndColorBar);
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &Widget::updateDepthImageAndColorBar);
    timer->start(1000);
    timer1 = new QTimer();  //用于返回信息处理
    QObject::connect(timer1, &QTimer::timeout, this,&Widget::readData);
    timer1->start(100);
    timer2 = new QTimer();
    //connect(ui->comboBox_Serial,SIGNAL(highlighted(int)),this,SLOT(Foundserialnum()));
    //设置串口状态标签为红色 表示未连接状态
}

//获取深度像素对应长度单位（米）的换算比例
float Widget::get_depth_scale(rs2::device dev)
{
    // Go over the device's sensors
    for (rs2::sensor& sensor : dev.query_sensors())
    {
        // Check if the sensor if a depth sensor
        if (rs2::depth_sensor dpt = sensor.as<rs2::depth_sensor>())
        {
            return dpt.get_depth_scale();
        }
    }
    throw std::runtime_error("Device does not have a depth sensor");
}

Mat Widget::align_Depth2Color(Mat depth,Mat color,rs2::pipeline_profile profile){
    //声明数据流
    auto depth_stream=profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream=profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth=depth_stream.get_intrinsics();
    const auto intrinColor=color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    //auto  extrinDepth2Color=depth_stream.get_extrinsics_to(color_stream);
    rs2_extrinsics  extrinDepth2Color;
    rs2_error *error;
    rs2_get_extrinsics(depth_stream,color_stream,&extrinDepth2Color,&error);

    //平面点定义
    float pd_uv[2],pc_uv[2];
    //空间点定义
    float Pdc3[3],Pcc3[3];

    //获取深度像素与现实单位比例（D415默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    int y=0,x=0;
    //初始化结果
    //Mat result=Mat(color.rows,color.cols,CV_8UC3,Scalar(0,0,0));
    Mat result=Mat(color.rows,color.cols,CV_16U,Scalar(0));
    //对深度图像遍历
    for(int row=0;row<depth.rows;row++){
        for(int col=0;col<depth.cols;col++){
            //将当前的(x,y)放入数组pd_uv，表示当前深度图的点
            pd_uv[0]=col;
            pd_uv[1]=row;
            //取当前点对应的深度值
            uint16_t depth_value=depth.at<uint16_t>(row,col);
            //换算到米
            float depth_m=depth_value*depth_scale;
            //将深度图的像素点根据内参转换到深度摄像头坐标系下的三维点
            rs2_deproject_pixel_to_point(Pdc3,&intrinDepth,pd_uv,depth_m);
            //将深度摄像头坐标系的三维点转化到彩色摄像头坐标系下
            rs2_transform_point_to_point(Pcc3,&extrinDepth2Color,Pdc3);
            //将彩色摄像头坐标系下的深度三维点映射到二维平面上
            rs2_project_point_to_pixel(pc_uv,&intrinColor,Pcc3);

            //取得映射后的（u,v)
            x=(int)pc_uv[0];
            y=(int)pc_uv[1];
            //            if(x<0||x>color.cols)
            //                continue;
            //            if(y<0||y>color.rows)
            //                continue;
            //最值限定
            x=x<0? 0:x;
            x=x>depth.cols-1 ? depth.cols-1:x;
            y=y<0? 0:y;
            y=y>depth.rows-1 ? depth.rows-1:y;

            result.at<uint16_t>(y,x)=depth_value;
        }
    }
    //返回一个与彩色图对齐了的深度信息图像
    return result;
}

void Widget::measure_distance(Mat &color,Mat depth,cv::Size range,rs2::pipeline_profile profile)
{
    //获取深度像素与现实单位比例（D415默认1毫米）
    float depth_scale = get_depth_scale(profile.get_device());
    qDebug()<<depth_scale;
    //定义图像中心点
    cv::Point center(color.cols/2,color.rows/2);
    //定义计算距离的范围
    cv::Rect RectRange(center.x-range.width/2,center.y-range.height/2,range.width,range.height);
    //遍历该范围
    float distance_sum=0;
    int effective_pixel=0;
    for(int y=RectRange.y;y<RectRange.y+RectRange.height;y++){
        for(int x=RectRange.x;x<RectRange.x+RectRange.width;x++){
            //如果深度图下该点像素不为0，表示有距离信息
            if(depth.at<uint16_t>(y,x)){
                distance_sum+=depth_scale*depth.at<uint16_t>(y,x);
                effective_pixel++;
            }
        }
    }
    qDebug()<<"遍历完成，有效像素点:"<<effective_pixel;
    float effective_distance=distance_sum/effective_pixel;
    qDebug()<<"目标距离："<<effective_distance<<" m";
    char distance_str[30];
    sprintf(distance_str,"the distance is:%f m",effective_distance);
    cv::rectangle(color,RectRange,Scalar(0,0,255),2,8);
    cv::putText(color,(String)distance_str,cv::Point(color.cols*0.02,color.rows*0.05),
                cv::FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),2,8);
}


void Widget::updateDepthImageAndColorBar()
{
    //realsenseD435 拍摄到的帧
    rs2::frameset frames;
    frames= p.wait_for_frames();
    //获取RGB图
    frame colorFrames = frames.get_color_frame();
    // 查询帧大小（宽度和高度）
    const int cw = colorFrames.as<video_frame>().get_width();
    const int ch = colorFrames.as<video_frame>().get_height();
    //帧转化为Mat
    cv::Mat colorImage = cv::Mat(cv::Size(cw, ch), CV_8UC3, (void*)colorFrames.get_data());
    //d435 是RGB模式 而 cv是 BGR模式 ，所以交换一下
    //cv::cvtColor(colorImage,colorImage,cv::COLOR_BGR2RGB);
    QImage image1(colorImage.data, colorImage.cols, colorImage.rows, colorImage.step, QImage::Format_RGB888);
    QGraphicsScene* scene1 = new QGraphicsScene();

    // 将QImage转换为QPixmap，并添加到场景中
    scene1->addPixmap(QPixmap::fromImage(image1));

    ui->graphicsView_2->setScene(scene1);
    ui->graphicsView_2->fitInView(scene1->sceneRect(), Qt::KeepAspectRatio);
    ui->graphicsView_2->show();
    //为深度数据的可视化显示深度着色器
    static colorizer color_map;

    frame depthFrames = frames.get_depth_frame();

    // 查询帧大小（宽度和高度）
    const int dw = depthFrames.as<video_frame>().get_width();
    const int dh = depthFrames.as<video_frame>().get_height();
    static frame color_depth_frames;
    //color_depth_frames = color_map.colorize(depthFrames);
    color_depth_frames = frames.get_depth_frame().apply_filter(color_map);


    // 从着色的深度数据中创建OpenCV大小（w，h）的OpenCV矩阵
    //Mat depthImage = Mat(Size(dw, dh), CV_8UC3, (void*)color_depth_frames.get_data());
    cv::Mat depthImage = cv::Mat(cv::Size(dw, dh), CV_16U, (void*)depthFrames.get_data());
    cv::Mat color_depthImage = cv::Mat(cv::Size(dw, dh), CV_8UC3, (void*)color_depth_frames.get_data());
    //cv::cvtColor(depthImage,depthImage,cv::COLOR_BGR2RGB);
    QImage image2(color_depthImage.data, color_depthImage.cols, color_depthImage.rows, color_depthImage.step, QImage::Format_RGB888);
    // 创建一个QGraphicsScene对象
    QGraphicsScene* scene = new QGraphicsScene();

    // 将QImage转换为QPixmap，并添加到场景中
    scene->addPixmap(QPixmap::fromImage(image2));

    ui->graphicsView->setScene(scene);
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    ui->graphicsView->show();

    //float depth_scale = get_depth_scale(profile.get_device());
    //float distance=depth_scale*depthImage.at<uint16_t>(351,682);
    //qDebug()<<distance<<depthImage.at<uint16_t>(351,682);
    Mat result=align_Depth2Color(depthImage,colorImage,profile);
    double maxDepth = 1000;  // 根据你的深度相机的规格来设置这个值
    Mat result_scaled,Color_result;
    result.convertTo(result_scaled, CV_8U, 255.0 / maxDepth);

    int x = 300;
    int y = 200;
    int width = 700;
    int height = 300;
    // 然后，我们使用颜色映射来给深度图像上色
    cv::applyColorMap(result_scaled, Color_result, cv::COLORMAP_JET);
    Color_result = Color_result(cv::Rect(x, y, width, height));
    imshow("1",Color_result);
    float depth_scale = get_depth_scale(profile.get_device());
    //float distance=depth_scale*depthImage.at<uint16_t>(351,682);

    //float distance2=depth_scale*result.at<uint16_t>(351,682);
    //qDebug()<<distance2;
    //measure_distance(colorImage,result,cv::Size(20,20),profile);
    //声明数据流
    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();

    //获取内参
    const auto intrinDepth = depth_stream.get_intrinsics(); //获取内参必须在或许相应帧之后，否则会报错
    const auto intrinColor = color_stream.get_intrinsics();

    //直接获取从深度摄像头坐标系到彩色摄像头坐标系的欧式变换矩阵
    const auto  extrinDepth2Color = depth_stream.get_extrinsics_to(color_stream);

    cv::Mat roi;
    cv::cvtColor(colorImage, roi, cv::COLOR_BGR2GRAY);
    roi = roi(cv::Rect(x, y, width, height));
    /*cv::Mat kernel = cv::getStructuringElement(MORPH_RECT,Size(3,3));
    cv::dilate(image,image,kernel);*/
    //cv::adaptiveThreshold(roi, roi, 255,cv::THRESH_BINARY_INV, cv::ADAPTIVE_THRESH_GAUSSIAN_C, 7,8);
    cv::threshold(roi, roi, 100, 255, cv::THRESH_BINARY);
    cv::imshow("image",roi);
    float start[2]={0,0};
    float step[2]={0,0};
    float final[2]={0,0};
    for(int i=0;i<height;i++)
    {
        if(roi.at<uchar>(height-1-i,width-1)==0)
        {
            start[0]=height-1-i;
            start[1]=width-1;
            break;
        }
    }
    qDebug()<<"循环起始点："<<start[0]<<","<<start[1];
    step[0]=start[0];
    step[1]=start[1];
    bool stop=0;
    bool found = 0;
    int k=0,j=0,last_k=2,last_j=2;
    while(!found)
    {
        for(k = -1; k <=1&& !stop; k++) {
            for(j = 3; j >= 0; j--) {
                if(roi.at<uchar>(step[0]+k,step[1]-j)==0 &&!(j==0&&k==0)&&!(last_k==-k&&last_j==0&&j==0)) //避免出现下一个还是原来的黑点或者下一个与这一个之间反复横跳
                {
                    step[0]=step[0]+k;
                    step[1]=step[1]-j;

                    last_k = k;
                    last_j = j;

                    stop=1;//表明找到了下一个黑色点，还没到线的末端,退出for循环，继续while循环
                    break;
                }
            }
        }
        if(!stop)
        {
            found=1; //表明没有找到下一个黑色点，退出while循环
        }
        stop = 0;
    }
    final[0]=step[0]+y;
    final[1]=step[1]+x;
    float distance_z = depth_scale*result.at<uint16_t>(final[0],final[1]);
    rs2_deproject_pixel_to_point(Camera_Location,&intrinDepth,final,distance_z);
    float distance = qSqrt(qPow(Camera_Location[0],2)+qPow(Camera_Location[1],2)+qPow(Camera_Location[2],2));
    while(distance == 0)
    {
        final[0]=final[0]-1;
        final[1]=final[1]+1;
        distance_z = depth_scale*result.at<uint16_t>(final[0],final[1]);
        rs2_deproject_pixel_to_point(Camera_Location,&intrinDepth,final,distance_z);
        distance = qSqrt(qPow(Camera_Location[0],2)+qPow(Camera_Location[1],2)+qPow(Camera_Location[2],2));
    }
    qDebug()<<"像素坐标为："<<step[1]<<","<<step[0]<<"对应距离为："<<distance<<"m";
    qDebug()<<"相机坐标为："<<Camera_Location[0]<<","<<Camera_Location[1]<<","<<Camera_Location[2];
    cv::Point3f srcPoint;  // 这是你的源点
    srcPoint.x = Camera_Location[0];
    srcPoint.y = Camera_Location[1];
    srcPoint.z = Camera_Location[2];
    // 将源点转换为齐次坐标
    cv::Mat srcPointMat = (cv::Mat_<double>(4, 1) << srcPoint.x, srcPoint.y, srcPoint.z, 1.0);

    // 应用变换矩阵
    cv::Mat dstPointMat = RT * srcPointMat;

    // 将目标点从齐次坐标转换回笛卡尔坐标
    cv::Point3f dstPoint(dstPointMat.at<double>(0, 0) / dstPointMat.at<double>(3, 0),
                         dstPointMat.at<double>(1, 0) / dstPointMat.at<double>(3, 0),
                         dstPointMat.at<double>(2, 0) / dstPointMat.at<double>(3, 0));
    qDebug()<<"世界坐标为："<<dstPoint.x<<","<<dstPoint.y<<","<<dstPoint.z;
    ui->doubleSpinBox_POSNOWX->setValue(dstPoint.x*100);
    ui->doubleSpinBox_POSNOWY->setValue(dstPoint.y*100);
    ui->doubleSpinBox_POSNOWZ->setValue(dstPoint.z*100);
}

Widget::~Widget()
{
    p.stop();
    delete ui;
}

bool Widget::eventFilter(QObject *obj, QEvent *event){

    bool choose_unchange =(event->type() ==  QEvent::MouseButtonPress ||
                            event->type() == QEvent::MouseButtonRelease ||event->type() == QEvent::KeyPress ||
                            event->type() == QEvent::KeyRelease ||event->type() == QEvent::MouseButtonDblClick);
    if(obj == ui->radioButton_State){
        if(choose_unchange){
            return true; // 过滤事件
        }
    }
    /*if(obj == ui->lineEdit_Forward || obj == ui->lineEdit_Backward)
    {
        if (event->type() == QEvent::KeyPress)
        {
            QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
            if (keyEvent->key() >= Qt::Key_0 && keyEvent->key() <= Qt::Key_9)
            {
              // 允许输入数字键
               return QObject::eventFilter(obj, event);
            }
            else if (keyEvent->key() == Qt::Key_Period)
            {
                // 允许输入小数点
                return QObject::eventFilter(obj, event);
            }
            else if (keyEvent->key() == Qt::Key_Backspace || keyEvent->key() == Qt::Key_Delete)
            {
                // 允许删除键删除文本
                return QObject::eventFilter(obj, event);
            }
            else
            {
                // 拦截其他按键事件
                return true;
            }
        }
    }*/
    return QWidget::eventFilter(obj, event);
}

void Widget::Serial_InitialSetting()
{
    //填充串口号组合框
    SearchSerialPorts();

    //填充串口波特率
    ui->comboBox_Baud->addItem("9600");
    ui->comboBox_Baud->addItem("14400");
    ui->comboBox_Baud->addItem("19200");
    ui->comboBox_Baud->addItem("28800");
    ui->comboBox_Baud->addItem("38400");
    ui->comboBox_Baud->addItem("56000");
    ui->comboBox_Baud->addItem("57600");
    ui->comboBox_Baud->addItem("115200");
    ui->comboBox_Baud->setCurrentIndex(7);

    //填充串口数据位
    ui->comboBox_Data->addItem("8位");
    ui->comboBox_Data->setCurrentIndex(0);

    //填充串口校验位
    ui->comboBox_Parity->addItem("无校验");
    ui->comboBox_Parity->addItem("奇校验");
    ui->comboBox_Parity->addItem("偶校验");
    ui->comboBox_Parity->setCurrentIndex(0);

    //填充串口停止位
    ui->comboBox_Stop->addItem("1位");
    ui->comboBox_Stop->addItem("2位");
    ui->comboBox_Stop->setCurrentIndex(0);
}

void Widget::SearchSerialPorts()
{
    //ui->comboBox_Serial->setEditable(false);
    //ui->comboBox_Serial->clearEditText();
    ui->comboBox_Serial->clear();
    QStringList Portstring;
    foreach(const QSerialPortInfo &info,QSerialPortInfo::availablePorts())
    {
        Portstring.append(info.portName());
        ui->comboBox_Serial->addItem(info.portName());
        qDebug() << "串口"<< info.portName() <<"搜索完成";
    }
    ui->comboBox_Serial->setCurrentIndex(-1);
}

void Widget::Control_InitialSetting()
{
    ui->comboBox_State_axial->addItem("前进");
    ui->comboBox_State_axial->addItem("后退");

    ui->comboBox_State_Radial->addItem("左转");
    ui->comboBox_State_Radial->addItem("右转");

    ui->doubleSpinBox_Axialspeed->setRange(0,1.00); //最小速度0.01cm/s，最大速度1cm/s
    ui->doubleSpinBox_Radialspeed->setRange(0,60.0); //最小速度0.1度/s，最大速度20度/s
    ui->doubleSpinBox_Backward->setRange(0,20.00); //最小距离0.01cm，最大距离20.00cm
    ui->doubleSpinBox_Forward->setRange(0,20.00); //最小距离0.1度，最大距离360度
    ui->doubleSpinBox_leftturn->setRange(0,360);
    ui->doubleSpinBox_rightturn->setRange(0,360);

    ui->textReceive->setReadOnly(true);
}

void Widget::on_pushButton_OpenorClose_clicked()
{
    if(ui->comboBox_Serial->currentIndex() == -1)
        QMessageBox::warning(this,"提示","未选择串口！");
    else
    {
        QSerialPort::DataBits dataBits = QSerialPort::Data8;
        QSerialPort::StopBits stopBits = QSerialPort::OneStop;
        QSerialPort::Parity parity = QSerialPort::NoParity;

        if (ui->pushButton_OpenorClose->text()=="打开串口")
        {
            serialPort->setPortName(ui->comboBox_Serial->currentText());
            serialPort->setBaudRate(ui->comboBox_Baud->currentText().toInt());
            //读取选择的停止位
            switch (ui->comboBox_Stop->currentIndex())
            {
            case 0:
                stopBits = QSerialPort::OneStop; break;
            case 1:
                stopBits = QSerialPort::TwoStop; break;
            }
            //读取选择的校验位
            switch (ui->comboBox_Parity->currentIndex())
            {
            case 0:
                parity = QSerialPort::NoParity; break;
            case 1:
                parity = QSerialPort::OddParity; break;
            case 2:
                parity = QSerialPort::EvenParity; break;
            }
            serialPort->setPortName(ui->comboBox_Serial->currentText());
            serialPort->setBaudRate(ui->comboBox_Baud->currentText().toInt());
            serialPort->setDataBits(dataBits);
            serialPort->setStopBits(stopBits);
            serialPort->setParity(parity);
            serialPort->setFlowControl(QSerialPort::NoFlowControl);
            if(serialPort->open(QIODevice::ReadWrite) == true)
            {
                QMessageBox::information(this,"提示","串口打开成功");
                ui->pushButton_OpenorClose->setText("关闭串口");
                ui->comboBox_Serial->setEnabled(false);
                ui->comboBox_Baud->setEnabled(false);
                ui->comboBox_Data->setEnabled(false);
                ui->comboBox_Stop->setEnabled(false);
                ui->comboBox_Parity->setEnabled(false);
                ui->radioButton_State->setChecked(true);
                ui->pushButton_Refresh->setEnabled(false);
            }
            else
                QMessageBox::critical(this,"提示","串口打开失败");
        }
        else
        {
            serialPort->close();
            QMessageBox::information(this,"提示","串口已关闭");
            ui->pushButton_OpenorClose->setText("打开串口");     //提示下次可以打开
            ui->comboBox_Serial->setEnabled(true);
            ui->comboBox_Baud->setEnabled(true);
            ui->comboBox_Data->setEnabled(true);
            ui->comboBox_Stop->setEnabled(true);
            ui->comboBox_Parity->setEnabled(true);
            ui->radioButton_State->setChecked(false);
            ui->pushButton_Refresh->setEnabled(true);
        }
    }
}

void Widget::on_pushButton_Refresh_clicked()
{
    SearchSerialPorts();
}

void Widget::readData()
{
    if (serialPort && serialPort->isOpen() && serialPort->isReadable() && serialPort->bytesAvailable() > 0)
    {
        QByteArray data = serialPort->readAll();
        QTextCodec* codec = QTextCodec::codecForName("GB2312");   //编码方式要与Keil中一致
        QString decodedString = codec->toUnicode(data);
        qint64 elapsedTime = E_timer.elapsed();
        // 在 QTextEdit、QLabel 或其他 Qt 小部件中显示文本数据
        qDebug()<<elapsedTime;
        ui->textEdit->setText(QString::number(elapsedTime));
        bool condition = decodedString == "轴向运动停止！\n" || decodedString == "径向运动停止！\n" || decodedString == "输入格式不正确！\n" || decodedString == "已到达轴向运动前限位！\n轴向运动停止！\n" || decodedString == "已到达轴向运动后限位！\n轴向运动停止！\n";
        if(condition)
        {
            ui->pushButton_Move->setEnabled(true);
        }
        ui->textReceive->append(decodedString);
        // 当限位通知到来时，写入串口的是数据要随限位的情况而有所选择，当成功写入后，要恢复限位标志到0
    }
}

void Widget::writeDone(QByteArray byteArray)
{
    int size = byteArray.size();
    if (serialPort && serialPort->isOpen() && serialPort->isWritable())
    {
        qint64 bytesWritten = serialPort->write(byteArray);
        if (bytesWritten == -1)
            QMessageBox::warning(this,"提示","数据发送失败");
        else if (bytesWritten == size)
        {
            //QMessageBox::information(this,"提示","数据发送成功");
            //ui->pushButton_Move->setEnabled(false);
        }
        else
            QMessageBox::warning(this,"提示","数据部分发送");
    }
    else
        QMessageBox::warning(this,"提示","串口未打开或在工作中");
}

void Widget::on_comboBox_State_axial_currentIndexChanged(int index)
{

    if (index)
    {
        ui->doubleSpinBox_Backward->setEnabled(true);
        ui->doubleSpinBox_Forward->setEnabled(false);
    }
    else
    {
        ui->doubleSpinBox_Forward->setEnabled(true);
        ui->doubleSpinBox_Backward->setEnabled(false);
    }
}

void Widget::on_comboBox_State_Radial_currentIndexChanged(int index)
{
    if (index)
    {
        ui->doubleSpinBox_rightturn->setEnabled(true);
        ui->doubleSpinBox_leftturn->setEnabled(false);
    }
    else
    {
        ui->doubleSpinBox_leftturn->setEnabled(true);
        ui->doubleSpinBox_rightturn->setEnabled(false);
    }
}

/*
    flag1表示轴向运动还是径向运动，flag1=0时为轴向运动，flag1=1时为径向运动;
    在flag1的基础上，flag2=0时为前进或左转；flag2=1时为后退或右转;
*/

int Widget::digits_compute(double number)
{
    double fraction = number - int(number);
    int integer = int(number);
    int digits;
    if (integer != 0)
        digits = (int)(log10(integer)) + 1;
    else if(fraction != 0)
        digits = floor(log10(fraction));
    else
        digits = 0;
    return digits;
}

struct ReturnData Widget::time_control(double speed,double distance) //轴向时间最长为20/0.01=2000s,最短为0.01/1=0.01s 我想让int_time无论多大，最后都是4位输出，最后带一个flag3
{  // 径向时间最长为720/0.1 = 7200s , 最短为0.1/10 = 0.01s 只要把预分频值设置为840，counter还是在32位范围内的
    struct ReturnData Data;
    if (speed != 0 && distance!=0)
    {
        double time = distance/speed; //单位 s
        int int_time;
        double fraction = time - int(time);
        int integer = int(time);
        int digits;  // 范围从0-5,分别表示4位到-2位。表示数字传输值到真实值，小数点需要左移的位数，也就是真实值到传输值，小数点需要右移的位数
        if (integer != 0)
            digits = abs((int)(log10(integer)) + 1-4);
        else
            digits = abs(floor(log10(fraction))-3);
        int flag;
        flag = 5 - digits;//表示在中断定时器10μs的基础上需要乘的10的多少次方，就可以从传输值转到定时器计数值，也就是说10μs的计数值counter*传输值*10^(flag3)就可以得到所需的新的中断计数器counter
        // 预分频值为84时，10μs对应counter=10。预分频值为840时，10μs对应counter=1.
        Data.flag = flag;
        int_time = int(time*pow(10,digits));
        Data.time = int_time;
    }
    else
    {
        Data.flag = 9;
        Data.time = 9999;
    }
    return Data;
}

/*void Widget::on_doubleSpinBox_Speed_valueChanged(double speed)
{
    if (speed>1.54)
    {
        if(speed>5.3)
        {
            speed=round(speed);
            ui->doubleSpinBox_Speed->setDecimals(0);
            ui->doubleSpinBox_Speed->setValue(speed);
        }
        else
        {
            speed = round(speed*10)/10;
            ui->doubleSpinBox_Speed->setDecimals(1);
            ui->doubleSpinBox_Speed->setValue(speed);
        }
    }
}
*/

/*void Widget::on_doubleSpinBox_Speed_editingFinished()
{
    double speed;
    speed = ui->doubleSpinBox_Speed->value();
    double Decimal = speed - int(speed);
    int secondDecimal =  int(Decimal*100) % 10;
    if (speed>1.54 && secondDecimal != 0 )
    {
        speed = round(speed*10)/10;
        ui->doubleSpinBox_Speed->setDecimals(1);
        ui->doubleSpinBox_Speed->setValue(speed);
        QMessageBox::information(this,"提示","数据精确度调整！");
    }
    ui->doubleSpinBox_Speed->setDecimals(2);
}*/


void Widget::on_pushButton_Move_clicked()
{
    //QByteArray byteArray = "";
    //QDataStream stream(&byteArray, QIODevice::WriteOnly);
    //stream.setVersion(QDataStream::Qt_5_0);
    int axial_index = ui->comboBox_State_axial->currentIndex();
    int radial_index = ui->comboBox_State_Radial->currentIndex();
    double axial_speed,axial_distance,radial_speed,radial_distance; //speed范围为0.01cm/s - 1 cm/s ,distance 范围为0.01cm - 20cm
    int int_axial_speed,int_axial_time,int_radial_speed,int_radial_time,flag1,flag2,flag3,flag4 ; //一个double是8字节(64位)，总共32字节(256位)
    // flag1表示前进(0)或后退(1)，flag2表示左转(0)或右转(1), flag3表示轴向运动对应时间的定时器计数值乘子，flag4表示径向运动对应时间的定时器计数值乘子。
    axial_speed = ui->doubleSpinBox_Axialspeed->value();
    radial_speed = ui->doubleSpinBox_Radialspeed->value();
    QByteArray data = "9";  // 数据发送起始检验位
    //data = serialdata_axial_speedfilter(axial_speed);
    switch(digits_compute(axial_speed))
    {
    case -2:
        data.append("00");break;
    case -1:
        data.append("0");break;
    case 1:
        break;
    case 0:
        data.append("00");break;
    }

    if (axial_index)
    {
        flag1 = 1 ;
        axial_distance = ui->doubleSpinBox_Backward->value();
    }
    else
    {
        flag1 = 0 ;
        axial_distance = ui->doubleSpinBox_Forward->value();
    }

    if(radial_index)
    {
        flag2 = 1;
        radial_distance = ui->doubleSpinBox_rightturn->value();
    }
    else
    {
        flag2 = 0;
        radial_distance = ui->doubleSpinBox_leftturn->value();
    }
    struct ReturnData axial_Data,radial_Data;
    axial_Data = time_control(axial_speed,axial_distance);
    int_axial_time = axial_Data.time;
    flag3 = axial_Data.flag;
    radial_Data = time_control(radial_speed,radial_distance);
    int_radial_time = radial_Data.time;
    flag4 = radial_Data.flag;
    int_axial_speed = int(axial_speed*100);//有符号int最大值是2147483647
    int_radial_speed = int(radial_speed*10);
    QByteArray serial_axial_speed = QByteArray::number(int_axial_speed);  //3位
    QByteArray serial_axial_time = QByteArray::number(int_axial_time);  //4位
    QByteArray serial_radial_speed = QByteArray::number(int_radial_speed);  //3位
    QByteArray serial_radial_time = QByteArray::number(int_radial_time); //4位
    QByteArray serial_flag1 = QByteArray::number(flag1); //1位
    QByteArray serial_flag2 = QByteArray::number(flag2); //1位
    QByteArray serial_flag3 = QByteArray::number(flag3); //1位
    QByteArray serial_flag4 = QByteArray::number(flag4); //1位
    data.append(serial_axial_speed);
    if (flag3 != 9)
        data.append(serial_axial_time);
    else
        data.append("0000");

    switch(digits_compute(radial_speed))
    {
    case -1:
        data.append("00");break;

    case 1:
        data.append("0");break;

    case 2:
        break;

    case 0:
        data.append("00");break;
    }

    data.append(serial_radial_speed);
    if (flag4 != 9)
        data.append(serial_radial_time);
    else
        data.append("0000");
    data.append(serial_flag1);
    data.append(serial_flag2);
    data.append(serial_flag3);
    data.append(serial_flag4);
    //stream << serial_speed << serial_distance << flag1 << flag2 ;
    qDebug()<< data <<data.size();
        // 总计18位
    if(flag3 != 9 || flag4 != 9)
    {
        writeDone(data);
        E_timer.start();
    }
    else
    {
        QByteArray data1 = "9"; // 起始位
        // 坐标运动时速度恒定为0.5cm/s
        data1.append("050");
        float posnow_x,posnow_y,posnow_z,posobj_x,posobj_y,posobj_z;
        posnow_x = ui->doubleSpinBox_POSNOWX->value();
        posnow_y = ui->doubleSpinBox_POSNOWY->value();
        posnow_z = ui->doubleSpinBox_POSNOWZ->value();
        posobj_x = ui->doubleSpinBox_POSOBJX->value();
        posobj_y = ui->doubleSpinBox_POSOBJY->value();
        posobj_z = ui->doubleSpinBox_POSOBJZ->value();

        struct ReturnData Posz_Data;
        Posz_Data = time_control(0.5,qAbs(posobj_z-posnow_z));
        int int_Posz_time = Posz_Data.time;
        int flag1;

        if(posnow_z < posobj_z)
        {
            flag1=0;
        }
        else
            flag1=1;

        int flag3 = Posz_Data.flag;
        QByteArray serial_Posz_time = QByteArray::number(int_Posz_time);  //4位
        QByteArray serial_flag1 = QByteArray::number(flag1); //1位
        QByteArray serial_flag3 = QByteArray::number(flag3); //1位
        data1.append(serial_Posz_time);
        data1.append("0000000");
        data1.append(serial_flag1);
        data1.append("0");
        data1.append(serial_flag3);
        data1.append("9");
        writeDone(data1);
    }
}

void Widget::on_pushButton_Clear_clicked()
{
    ui->textReceive->clear();
}

