
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "QtDebug"
#include <QtCore>


void MainWindow::getMinMax(pair *minmax, signed short *arr, int n)
{
  float mid;
  int i;

  //If there is only one element then return it as min and max both
  if (n == 1)
  {
     minmax->max = arr[0];
     minmax->min = arr[0];
     return ;
  }

  //If there are more than one elements, then initialize min
  //    and max

  if (arr[0] > arr[1])
  {
      minmax->max = arr[0];
      minmax->min = arr[1];

  }
  else
  {
      minmax->max = arr[1];
      minmax->min = arr[0];
  }

  mid = arr[0] + arr[1];

  for (i = 2; i<n; i++)
  {
      mid += arr[i];
    if (arr[i] >  minmax->max)
      minmax->max = arr[i];

    else if (arr[i] <  minmax->min)
      minmax->min = arr[i];
  }

  minmax->mid = mid/n;

}


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    // Download calibrate data 25 deg C
    FILE *fp= fopen("0.bin", "rb");
    if(fp != NULL){
      fread(image_cal, 2, 384*288, fp);
      fclose(fp);
    }

    therm = thermapp_initUSB();
    if(therm == NULL) {
        qDebug()<< "init Error";
    }

    /// Debug -> check for thermapp
    if(thermapp_USB_checkForDevice(therm, VENDOR, PRODUCT) == -1){
       qDebug()<< "USB_checkForDevice Error";
    }else
    {
        qDebug()<< "thermapp_FrameRequest_thread";
        //Run thread usb therm
        thermapp_FrameRequest_thread(therm);
    }

    /// Run frame refresh timer
    frameTimer = new QTimer(this);
    connect(frameTimer, SIGNAL(timeout()), this, SLOT(UpdateProcessFrame()));
    frameTimer->start(100);
    //thermapp_setGain(therm,0);


    // wfp = fopen("gain0.txt", "w");
    temp_prev = thermapp_getTemperature(therm);

    count = 0;

    //on_GainSlider_valueChanged(gain_kof[0]);

    //sTimer = new QTimer(this);
    //connect(sTimer, SIGNAL(timeout()), this, SLOT(sample()));
    //sTimer->start(600);


    ui->VoutA_Slider->setValue(therm->cfg->VoutA);
    ui->VoutC_Slider->setValue(therm->cfg->VoutC);
    ui->VoutD_Slider->setValue(therm->cfg->VoutD);
    ui->VoutE_Slider->setValue(therm->cfg->VoutE);


    ui->L_VoutA->setText(QString("VoutA: %1").arg(((float)2.5/2048) * therm->cfg->VoutA));
    ui->L_VoutC->setText(QString("VoutC: %1").arg(((float)2.5/2048) * therm->cfg->VoutC));
    ui->L_VoutD->setText(QString("VoutD: %1").arg(((float)2.5/2048) * therm->cfg->VoutD));
    ui->L_VoutE->setText(QString("VoutE: %1").arg(((float)2.5/2048) * therm->cfg->VoutE));

    gainCal = 1;
    ui->gainLabel->setText(QString("%1").arg(1));
    offsetCal = 0;
    ui->offsetLabel->setText(QString("%1").arg(0));

    ui->spinBox->setValue(2);
}

MainWindow::~MainWindow()
{
    thermapp_Close(therm);
    delete ui;
}


///FIXME: This method transfers RAW data from ThermApp to QPixmap
QPixmap MainWindow::Frame(short *frame){

   int pix_lim, i, tmp = 0, tmp0 = 0;
   //double pix;
   // unsigned short gain = (unsigned short)thermapp_getGain(therm);
   //int i_f=0;
   int i_p=0;

   short frame_trans[PIXELS_DATA_SIZE];

    for(i = 0; i < PIXELS_DATA_SIZE; i++){
        frame_trans[i] = ((frame[i] - image_cal[i]) * gainCal) + offsetCal;
    }

  pair min_max ;
  getMinMax(&min_max, frame_trans, PIXELS_DATA_SIZE);
  ui->labelDinamic->setText(QString("Dynamic range: %1").arg(sqrt((min_max.max - min_max.min) * (min_max.max - min_max.min))));
  ui->minmaxlabel->setText(QString("max: %1, min: %2").arg(min_max.max).arg(min_max.min));
    for(i = 0; i < PIXELS_DATA_SIZE; i++){

        pix_lim = frame_trans[i];// * gainCal;


        //qDebug() << pix_lim;
        if((pix_lim > 255) && (tmp == 0)){
            tmp = 1;
            qDebug() << "overflow > " << i << pix_lim;
            //int agc = ui->gainCalSlider->value();
            //agc -= 50;
            //ui->gainCalSlider->setValue(agc);

        }

        if((pix_lim < 0) && (tmp0 == 0)){
            tmp0 = 1;
            //int agc = ui->offsetCalSlider->value();
            qDebug() << "overflow < "<< i << pix_lim;
            //agc += sqrt(min_max.min * min_max.min);
            //ui->offsetCalSlider->setValue(agc);
        }


        if(pix_lim > 255)
            pix_lim = 255;

        if(pix_lim < 0){
            //qDebug() << "overflow <";
            pix_lim = 0;
        }
        imageData[i_p] = (unsigned char)pix_lim;
        i_p++;
        imageData[i_p] = (unsigned char)pix_lim;
        i_p++;
        imageData[i_p] = (unsigned char)pix_lim;
        i_p++;

    }

    QPixmap pixmap = QPixmap::fromImage(
        QImage(
            (unsigned char *) imageData,
            384,
            288,
            QImage::Format_RGB888
                   // ,QImage::Format_Mono
        )
    );
    return pixmap.transformed(QTransform().scale(-2, 2));
}



void MainWindow::UpdateProcessFrame(){
    //while(1);

    //qDebug()<<  "in ----->>> UpdateProcessFrame()";
    short frame[PIXELS_DATA_SIZE];
    if(thermapp_GetImage(therm, frame))
    {

        //qDebug() << "temp: " << thermapp_getTemperature(therm) << " pixel: " << pixel0 <<" radiation: "<< ui->lineEdit->text();
        ui->scen->setPixmap(Frame(frame));
        ui->id->setText(QString("%1").arg(thermapp_getId(therm)));
        ui->frames->setText(QString("%1").arg(thermapp_getFrameCount(therm)));
        // qDebug()<<  thermapp_getFrameCount(therm);
        //printf("%d\n", thermapp_getFrameCount(therm));
        ui->temp->setText(QString("%1").arg(thermapp_getTemperature(therm)));
        ui->error_frames->setText(QString("%1").arg(therm->lost_packet));
    }
   // qDebug()<<  "out ----->>> UpdateProcessFrame()";

}

void MainWindow::sample(){

}

// Get Calibrate data
void MainWindow::on_getCalibButton_pressed()
{
    frameTimer->stop();
    short frame[PIXELS_DATA_SIZE];
    double d_frame[PIXELS_DATA_SIZE];

    while(!thermapp_GetImage(therm, frame));

    for(int i = 0; i < PIXELS_DATA_SIZE; i++){
        d_frame[i] = frame[i];
    }

    for(int i = 0; i < 50; i++){
        while(!thermapp_GetImage(therm, frame));

        for(int j = 0; j < PIXELS_DATA_SIZE; j++){
            d_frame[j] += frame[j];
        }
    }

    for(int i = 0; i < PIXELS_DATA_SIZE; i++){
         image_cal[i] = d_frame[i] / 50;
    }

    frameTimer->start(100);


}

void MainWindow::on_setCalibButton_pressed()
{

}



void MainWindow::on_VoutA_Slider_valueChanged(int value)
{
    ui->L_VoutA->setText(QString("VoutA: %1").arg(((float)2.5/2048) * value));
    therm->cfg->VoutA = value;
}

void MainWindow::on_VoutC_Slider_valueChanged(int value)
{
    ui->L_VoutC->setText(QString("VoutC: %1").arg(((float)2.5/2048) * value));
    therm->cfg->VoutC = value;
}

void MainWindow::on_VoutD_Slider_valueChanged(int value)
{
    ui->L_VoutD->setText(QString("VoutD: %1").arg(((float)2.5/2048) * value));
    therm->cfg->VoutD = value;
}

void MainWindow::on_VoutE_Slider_valueChanged(int value)
{
    ui->L_VoutE->setText(QString("VoutE: %1").arg(((float)2.5/2048) * value));
    therm->cfg->VoutE = value;
}


/// Debug  some test
void MainWindow::on_horizontalSlider_3_valueChanged(int value)
{
#if 0
    int temp = (value<<16)&0xffff0000;
#else
    int temp = (value)&0x0000ffff;
#endif
  //  therm->cfg->none_volatile_data5 = temp;
    ui->label_3->setText(QString("%1").arg(temp));
}

/// Debug
void MainWindow::on_gainSlider_valueChanged(int value)
{
    gainCal = (double)value/32768;
    ui->gainLabel->setText(QString("%1").arg(gainCal));

}

void MainWindow::on_offsetSlider_valueChanged(int value)
{
    offsetCal = value;
    ui->offsetLabel->setText(QString("%1").arg(offsetCal));
}

void MainWindow::on_spinBox_valueChanged(int arg1)
{
    therm->cfg->modes = arg1 & 0x000f;
}

void MainWindow::on_clearButton_pressed()
{
    memset(image_cal, 0, sizeof(image_cal));
}
