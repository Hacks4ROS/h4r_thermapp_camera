#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include <QtGui>
#include "thermapp.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QPixmap Frame(short *frame);
    void GenTable();

    typedef struct _pair
    {
        signed short min;
        signed short mid;
        signed short max;
    }pair;

    void getMinMax(pair *minmax, signed short *arr, int n);

public slots:
     void UpdateProcessFrame();

     void sample();

private slots:
     void on_getCalibButton_pressed();

     void on_setCalibButton_pressed();


     void on_VoutA_Slider_valueChanged(int value);

     void on_VoutC_Slider_valueChanged(int value);

     void on_VoutD_Slider_valueChanged(int value);

     void on_VoutE_Slider_valueChanged(int value);

     void on_horizontalSlider_3_valueChanged(int value);

     void on_gainSlider_valueChanged(int value);

     void on_offsetSlider_valueChanged(int value);

     void on_spinBox_valueChanged(int arg1);

     void on_clearButton_pressed();

private:
    Ui::MainWindow *ui;
    ThermApp *therm;
    QTimer *frameTimer;
    unsigned char imageData[PIXELS_DATA_SIZE*3];

    short image_cal[384*288]; //Calibrate data
    float temp_prev;

    int DCoffset;
    short pixel0;

    //FILE *wfp;

   QTimer *sTimer;

    float Tambient;
    short PixL[11], PixM[11], PixH[11];
    short PixL_,PixM_,PixH_;
    int count;

    double gainCal;
    int offsetCal;

};

#endif // MAINWINDOW_H
