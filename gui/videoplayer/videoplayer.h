
#ifndef  _VIDEOPLAYER_H
#define _VIDEOPLAYER_H

#include <QWidget>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <memory>
#include "videoplayer_exports.h"
#include <QButtonGroup>
#include <QBoxLayout>
#include <QScrollArea>
#include "aruco/aruco.h"
#include "cameracontrols.h"
#include "genericimagereader.h"
class QAbstractButton;
class QSlider;
class QLabel;
class QUrl;
 class GenericImageReader;
 class SceneImageViewer;
class APP_VIDEOPLAYER_TOOLS_API VideoPlayer : public QWidget
{
    Q_OBJECT
public:
    enum TYPES : int {TYPE_NONE,TYPE_IMAGE,TYPE_VIDEO,TYPE_LIVE};
    VideoPlayer(QWidget *parent = nullptr);
    ~VideoPlayer();
    void addButton(QAbstractButton*);
    int getType() ;


    cv::Mat getInputImage(){return imIn;}
    cv::Mat getShownImage(){return imshown;}
    std::string getCurrentImageInfo();
    void setProcessImageWithArucoDetector(bool v){_processImageWithArucoDetector=v;};
    bool processImageWithArucoDetector()const{return _processImageWithArucoDetector;}
    std::vector<aruco::Marker> getDetectedMarkers(){return detectedMarkers;}
    bool isVideo()const;
    QStringList getSource()const;
    int getFrameIndex()const;


    std::vector<QAction*> getActions(){return _actions;}
    void showAllCandidates(bool v);
    void retrieveAndShow();
 public slots:
    void reset();
    void on_deactivate (  );
    void on_activate (  );
    void openVideoFile();
    bool openVideoFilePath(QString path);
    void openImages();
    void openVideoCamera(bool toogled);
    void setImage(  cv::Mat &img2Show);
    void updateImage();
    bool playNextFrame();
    void releaseReader();

    void cameraReconnectRequest(int idx,QString resolution);

    void setMaxSpeed(bool yesno){_maxSpeed=yesno;}
private slots:
    void setSliderPos(int pos);
    void playPauseButtonClicked( );
    void sliderReleased( );
    void valueChanged(int);
    void closeCamera();
    void saveCurrentImage();
signals:
    void newImage(cv::Mat &);
    void openedImageOrVideo();
protected:

private:
    QAbstractButton *m_playButton;

    cameracontrols *camcontrols;
     std::vector<QAction*> _actions;
     QAction *cameraAction;

    std::vector<QAbstractButton *> addedButtons;
    QSlider *m_positionSlider;
    QLabel *m_errorLabel;
    QLabel *frameNumberLabel;
    cv::Mat imIn,imshown;
    SceneImageViewer *imageWdgt;
    QBoxLayout *controlLayout;
    QScrollArea *scrollArea;
    int isPlaying=0;
    std::shared_ptr<GenericImageReader> _reader;
    std::vector<aruco::Marker> detectedMarkers;
    bool _processImageWithArucoDetector=false;
    void prepareForOpenedReader();
    bool grabAndShow();
    float scaleFactor=1;


    bool _showAllCandidates=false;
    bool _maxSpeed=false;
}; 

#endif
