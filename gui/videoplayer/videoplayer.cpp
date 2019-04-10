#include "videoplayer.h"


#include <QtGui>
#include <QLabel>
#include <QPushButton>
#include <QStyle>
#include <QSlider>
#include <QString>
#include <QAction>
#include <QFileDialog>
#include <QScrollBar>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstring>
#include "global_gparams.h"

#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QSizePolicy>
#include <QMessageBox>
#include <thread>
class SceneImageViewer : public QLabel
{

public:
    explicit SceneImageViewer(QWidget *parent = 0):
        QLabel(parent)
    {
        this->setMinimumSize(1,1);
        setScaledContents(false);
    }
    virtual int heightForWidth( int width ) const{
        return pix.isNull() ? this->height() : ((qreal)pix.height()*width)/pix.width();

    }
    virtual QSize sizeHint() const{
        int w = this->width();
        return QSize( w, heightForWidth(w) );
    }
    QPixmap scaledPixmap() const{
        return pix.scaled(this->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation);
    }
    void setImage(const cv::Mat &img2Show) {
                QImage _qimgR ( ( const uchar * ) ( img2Show.ptr<uchar> ( 0 ) ),
                                img2Show.cols,img2Show.rows, QImage::Format_RGB888 ) ;


                setPixmap ( QPixmap::fromImage ( _qimgR.rgbSwapped() ) );

    }
public slots:
    void setPixmap ( const QPixmap & p)
    {
        pix = p;
        QLabel::setPixmap(scaledPixmap());
    }
    void resizeEvent(QResizeEvent *e){
        if(!pix.isNull())
            QLabel::setPixmap(scaledPixmap());
    }
private:
    QPixmap pix;
};


int VideoPlayer::getType(){
    if(!_reader)return TYPE_NONE;
    else if ( dynamic_cast<_ImagePlayer*>( _reader.get())!=0)return TYPE_IMAGE;
    else if ( dynamic_cast<_VideoPlayer*>( _reader.get())!=0)return TYPE_VIDEO;
    else if ( dynamic_cast<_CameraPlayer*>( _reader.get())!=0)return TYPE_LIVE;
    return TYPE_NONE;
}  ;

VideoPlayer::VideoPlayer(QWidget *parent)
    : QWidget(parent)
{

    imageWdgt=new SceneImageViewer();
    imageWdgt->setPixmap(QPixmap ( QString:: fromUtf8 ( ":/images/cityoflove.jpg" ) ));

    _actions.push_back( new QAction ( QIcon ( ":/images/movie.png" ), tr ( "&Open Video..." ), this ));
    connect(_actions.back(),&QAction::triggered,this,&VideoPlayer::openVideoFile);



    _actions.push_back( new QAction ( QIcon ( ":/images/open.png" ), tr ( "&Open Image(s)..." ), this ));
    connect(_actions.back(),&QAction::triggered,this,&VideoPlayer::openImages);


    _actions.push_back( new QAction ( QIcon ( ":/images/camera_color.png" ), tr ( "&Connect to Camera..." ), this ));
    connect(_actions.back(),&QAction::triggered,this,&VideoPlayer::openVideoCamera);
    _actions.back()->setCheckable(true);
    cameraAction=_actions.back();



    _actions.push_back( new QAction ( QIcon ( ":/images/save.png" ), tr ( "&Save Image..." ), this ));
    connect(_actions.back(),&QAction::triggered,this,&VideoPlayer::saveCurrentImage);

    m_playButton = new QPushButton;
    m_playButton->setEnabled(false);
    m_playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    connect(m_playButton, &QAbstractButton::clicked,this, &VideoPlayer::playPauseButtonClicked);

    camcontrols=new cameracontrols(0);
    connect(camcontrols,&cameracontrols::reconnect,this,&VideoPlayer::cameraReconnectRequest);


    m_positionSlider = new QSlider(Qt::Horizontal);
    m_positionSlider->setRange(0, 0);

    connect(m_positionSlider, &QAbstractSlider::sliderReleased,this, &VideoPlayer::sliderReleased);
    connect(m_positionSlider, &QAbstractSlider::valueChanged,this, &VideoPlayer::valueChanged);

    m_errorLabel = new QLabel;
    m_errorLabel->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    controlLayout = new QHBoxLayout;
    controlLayout->setMargin(0);

    controlLayout->addWidget(m_playButton);
    controlLayout->addWidget(m_positionSlider);
    frameNumberLabel=new QLabel(this);
    controlLayout->addWidget(frameNumberLabel);


    m_playButton->hide();
    m_positionSlider->hide();
   frameNumberLabel->hide();

    QBoxLayout *layout = new QVBoxLayout;



    layout->addWidget(camcontrols);
    camcontrols->hide();
     layout->addWidget(imageWdgt);
    layout->addLayout(controlLayout);
    layout->addWidget(m_errorLabel);



    setLayout(layout);


 }


VideoPlayer::~VideoPlayer()
{
}
QStringList VideoPlayer::getSource()const{
    if ( !_reader)return{};
    else return _reader->getSource();
}

void VideoPlayer::addButton(QAbstractButton* btn){
    controlLayout->addWidget(btn);
    addedButtons.push_back(btn);


}

void VideoPlayer::saveCurrentImage(){
    if(imIn.empty())return;
    QSettings settings;
    QString file = QFileDialog::getSaveFileName (
                this,
                tr ( "Select an output file" ),
                settings.value ( "currDir" ).toString(),
                tr ( "Image (*.jpg *.png)" ) );
    if ( file==QString() ) return;
    settings.setValue ( "currDir",QFileInfo ( file ).absolutePath() );
   cv::imwrite(file.toStdString(),imIn);
}


bool VideoPlayer::isVideo()const{
    if(!_reader)return false;
    if ( dynamic_cast<_VideoPlayer*>( _reader.get())!=0)return true;
    return false;
}

void VideoPlayer::openVideoFile(){
    QSettings settings;
    QString file = QFileDialog::getOpenFileName (
                this,
                tr ( "Select an input file" ),
                settings.value ( "currDir" ).toString(),
                tr ( "Open Movie (*.*)" ) );
    if ( file==QString() ) return;
    settings.setValue ( "currDir",QFileInfo ( file ).absolutePath() );
    openVideoFilePath(file);


}

bool VideoPlayer::openVideoFilePath(QString file){
    closeCamera();
    releaseReader();

    scaleFactor=1;
    _reader=std::make_shared<_VideoPlayer>(file);

    if (!_reader->isOpened()){
        m_errorLabel->setText(tr("Could not open file ")+file);
        return false;
    }
    prepareForOpenedReader();
    return true;
}


void VideoPlayer::closeCamera(){
    cameraAction->setChecked(false);
    camcontrols->hide();
}
void VideoPlayer::openVideoCamera(bool toggled){
    if(!toggled){
        releaseReader();
        imageWdgt->setPixmap(QPixmap ( QString:: fromUtf8 ( ":/images/cityoflove.jpg" ) ));
        camcontrols->hide();
    }
    else{
        scaleFactor=1;
        releaseReader();
         _reader=std::make_shared<_CameraPlayer>();
         camcontrols->set(_reader);
         camcontrols->show();
         prepareForOpenedReader();
    }
}
void VideoPlayer::reset(){
    releaseReader();
    imageWdgt->setPixmap(QPixmap ( QString:: fromUtf8 ( ":/images/cityoflove.jpg" ) ));
}

void VideoPlayer::cameraReconnectRequest(int idex,QString resolution){

    string str=resolution.toStdString();
    for(auto &c:str)
        if( c=='x') c=' ';
    stringstream sstr;sstr<<str;
    int w,h;
    sstr>>w>>h;
    isPlaying=false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    _reader.reset();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    _reader=std::make_shared<_CameraPlayer>(idex,w,h);
    _reader->grab();
    cv::Mat imaux;
    _reader->retrieve(imaux);
    if(imaux.cols!=w || imaux.rows!=h){
        _reader.reset();
        QMessageBox::critical (0,tr ( "Error" ),tr ( "Invalid resolution" ));
        return;
    }
    isPlaying=true;
    playNextFrame();
}


void VideoPlayer::openImages()
{
    closeCamera();
    releaseReader();
    scaleFactor=1;
    QSettings settings;
    QStringList files = QFileDialog::getOpenFileNames (
                this,
                tr ( "Select one or several images" ),
                settings.value ( "currDir" ).toString(),
                tr ( "Open Images (*.*)" ) );
    if ( files.size()==0) return;

    settings.setValue ( "currDir",QFileInfo ( files.at(0) ).absolutePath() );

    m_errorLabel->clear();
    _reader=std::make_shared<_ImagePlayer>(files);
    if (!_reader->isOpened()){
        m_errorLabel->setText(tr("Could not open image files"));
        return;
    }
    prepareForOpenedReader();
}
void VideoPlayer::on_deactivate(){
    if(isPlaying){
        isPlaying=false;
    }

}
void VideoPlayer::on_activate (  ){
     if(getType()==TYPE_LIVE)
     {
         isPlaying=true;
         playNextFrame();
     }
}

void VideoPlayer::prepareForOpenedReader(){


    emit     openedImageOrVideo();

    m_positionSlider->setPageStep(_reader->get(CV_CAP_PROP_FRAME_COUNT)>=40?10:1);
    m_positionSlider->setRange(0, _reader->get(CV_CAP_PROP_FRAME_COUNT)-1);
    m_positionSlider->setTickInterval(_reader->get(CV_CAP_PROP_FRAME_COUNT)>=40?_reader->get(CV_CAP_PROP_FRAME_COUNT)/10:1);
     m_positionSlider->setTickPosition(QSlider::TicksBelow);
    if (_reader->get(CV_CAP_PROP_FRAME_COUNT)>1  ){
        m_playButton->show();
        m_positionSlider->show();
        frameNumberLabel->show();
    }
    else{
        m_playButton->hide();
        m_positionSlider->hide();
        frameNumberLabel->hide();
    }

    m_playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    m_playButton->setEnabled(true);
    isPlaying=0;
    setSliderPos(0);
    if( getType()==TYPE_LIVE){//live video  camera
        m_playButton->show();

       playPauseButtonClicked();
    }
    else
        grabAndShow();

}


std::string VideoPlayer::getCurrentImageInfo(){
    if (!_reader) return "";
    return _reader->getInfo();
}


void VideoPlayer::playPauseButtonClicked()
{
    if (!isPlaying){
        m_playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
        isPlaying=1;
        playNextFrame();
    }
    else{
        m_playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
        isPlaying=0;
    }
}
void VideoPlayer::releaseReader(){
    if(isPlaying){
        isPlaying=false;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    _reader.reset();
    m_playButton->hide();
    m_positionSlider->hide();
    frameNumberLabel->hide();

    m_playButton->hide();
}
int VideoPlayer::getFrameIndex()const{
    if(!_reader)return -1;
    return _reader->get(CV_CAP_PROP_POS_FRAMES);
}

bool VideoPlayer::playNextFrame(){
    if(!_reader)return false;
    bool res=grabAndShow();

    if(res && isPlaying){
        if(_maxSpeed)
            QTimer::singleShot((1000./_reader->get(CV_CAP_PROP_FPS))-2,this,SLOT(playNextFrame()));
        else
            QTimer::singleShot(5,this,SLOT(playNextFrame()));
        }
    else{
        isPlaying=0;
        m_playButton->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    }
    return res;
}

void VideoPlayer::updateImage(){

    if (imIn.empty()) return ;
    imIn.copyTo(imshown);
    if (_processImageWithArucoDetector){
        detectedMarkers=ArucoMarkerDetector::get().detect(imshown);
        for(auto m:detectedMarkers) m.draw(imshown);
    }
    setImage(imshown);

}


bool VideoPlayer::grabAndShow(){
    if(!_reader)return false;
    m_positionSlider->blockSignals(true);
    setSliderPos(_reader->get(CV_CAP_PROP_POS_FRAMES));
    m_positionSlider->blockSignals(false);
    if (!_reader->grab()) return false;
    _reader->retrieve(imIn);

     emit newImage(imIn);


    if (imIn.empty()) return false;
    imIn.copyTo(imshown);

    if (_processImageWithArucoDetector){
        detectedMarkers=ArucoMarkerDetector::get().detect(imshown);
        for(auto m:detectedMarkers) m.draw(imshown);
    }
    setImage(imshown);
    return true;
}

void VideoPlayer::retrieveAndShow(){
    if(!_reader)return  ;
    m_positionSlider->blockSignals(true);
    setSliderPos(_reader->get(CV_CAP_PROP_POS_FRAMES));
    m_positionSlider->blockSignals(false);
   // if (!_reader->grab()) return false;
    _reader->retrieve(imIn);

     emit newImage(imIn);


    if (imIn.empty()) return  ;
    imIn.copyTo(imshown);

    if (_processImageWithArucoDetector){
        detectedMarkers=ArucoMarkerDetector::get().detect(imshown);
        for(auto m:detectedMarkers) m.draw(imshown);
    }
    setImage(imshown);
    return  ;
}

void VideoPlayer::setSliderPos(int value){
    m_positionSlider->setValue(value);
    QString num;num.setNum(value);
    frameNumberLabel->setText("frame:"+num);
}


void VideoPlayer::valueChanged(int value){
    _reader->set(CV_CAP_PROP_POS_FRAMES,m_positionSlider->value());
    grabAndShow();
}
void VideoPlayer::sliderReleased(){
}


void VideoPlayer::setImage(  cv::Mat &img2Show){

    imageWdgt-> setImage ( img2Show );
}




