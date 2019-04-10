#ifndef CALIBRATIONCONTROLPANEL_H
#define CALIBRATIONCONTROLPANEL_H

#include <QWidget>
#include <QListWidgetItem>
#include <QLabel>
#include <QThread>
#include "aruco/aruco.h"
#include <opencv2/core/core.hpp>

namespace Ui {
class calibrationControlPanel;
}
 class calibrationControlPanel : public QWidget
{
    Q_OBJECT

public:
    explicit calibrationControlPanel(QWidget *parent = 0);
    ~calibrationControlPanel();

    void add(const cv::Mat &image,const std::vector<aruco::Marker> &markers,QString info);
private slots:
    void on_pb_images_remove_clicked();

    void on_pb_images_clear_clicked();

    void on_listWidget_itemDoubleClicked(QListWidgetItem *item);

    void on_pb_compute_clicked();

    void on_pb_saveCalibration_clicked();


  public:
    struct ImageInfo{
      cv::Mat image;
        std::vector<aruco::Marker> markers;
        QString info;
    };
private:
    Ui::calibrationControlPanel *ui;
    QLabel *shownImage ;

    std::list<ImageInfo> ImagesMarkers;
    aruco::CameraParameters camParams;

};


#endif // CALIBRATIONCONTROLPANEL_H
