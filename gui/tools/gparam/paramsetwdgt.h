#ifndef PARAMSETWDGT_H
#define PARAMSETWDGT_H

#include <QWidget>
#include <QFormLayout>
#include <QLabel>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QFileDialog>
#include <QVBoxLayout>
#include <QDialogButtonBox>
#include "gparam.h"
#include "gparam_exports.h"
namespace gparam
{
class GPARAM_API ParamSetWdgt : public QWidget
{
    Q_OBJECT
public:
    explicit ParamSetWdgt ( QWidget *parent = nullptr, QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::NoButton );
    explicit ParamSetWdgt ( const gparam::ParamSet &paramSet,QWidget *parent = 0, QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::NoButton );
    explicit ParamSetWdgt ( gparam::ParamSet *paramSet,QWidget *parent = 0, QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::NoButton );
    ~ParamSetWdgt();
    /**Works on a internal copy of the data
     */
    void setParamSet ( const gparam::ParamSet &paramSet );
    /**Works on a the paramset passed
     */
    void setParamSet ( gparam::ParamSet *paramSet );
    gparam::ParamSet getParamSet() const
    {
        return *_paramSet;
    };
    
    void setButtons(QDialogButtonBox::StandardButtons buttons) {
      _buttons = buttons;
      createButtonBox(buttons);
    };

    void setFileDialogPath(QString path);
private:
    void createInitialElements();
    void createButtonBox ( QDialogButtonBox::StandardButtons buttons );

    void addElement(QFormLayout *layout,const Param &p,int row=-1);
     void addInteger (QFormLayout *layout, gparam::Param intParam,int pos );
    void addString (QFormLayout *layout, gparam::Param strParam ,int pos);
    void addStringList (QFormLayout *layout, gparam::Param strlistParam ,int pos);
    void addReal (QFormLayout *layout, gparam::Param realParam ,int pos);
    void addBoolean ( QFormLayout *layout,gparam::Param boolParam ,int pos);
    void addFilepath ( QFormLayout *layout,gparam::Param filepathParam ,int pos);
    void addAdvancedCheckBox();

    void storeChange ( int idx, QObject* obj );

    void clear();


    void createLayouts();
    QCheckBox *advancedCheckBox=nullptr;
    std::vector<QWidget*> elements;
     QVBoxLayout *_bLayout=nullptr;
     QWidget *_normalcontrols=0,*_extracontrols=0;
     QFormLayout *_buttonlayout=0;
    gparam::ParamSet *_paramSet=0;
    gparam::ParamSet internal_paramSet;
    QDialogButtonBox* _buttonBox=0;
    QDialogButtonBox::StandardButtons _buttons;
    QString fileDlgOpenPath;
 signals:
    void paramChanged ( int idx );
    void okBtnPressed();
    void cancelBtnPressed();
    void applyBtnPressed();
    void advancedParamsToogled( bool );

private slots:
    void advancedToggled(bool checked);
    void okPressed();
    void cancelPressed();
    void applyPressed(); 
    void findSignalSender();
    void paramSetModified();
};



class QFilePathLineEdit : public QLineEdit
{
public:
    QFilePathLineEdit ( QWidget *parent = 0 )
    {
        _button = new QPushButton ( "..." );
        _button->setParent ( this );
        _fileDialog = new QFileDialog();
	_fileDialog->setModal(true);
        this->setTextMargins ( 0,0,30,0 ); // not write behind button
        connect ( _button, SIGNAL ( clicked() ), _fileDialog, SLOT ( show() ) );
        connect ( _fileDialog, SIGNAL ( fileSelected ( QString ) ), this, SLOT ( setText ( QString ) ) );
    }
    QFileDialog* getFileDialog()
    {
        return _fileDialog;
    }

protected:
    void resizeEvent ( QResizeEvent *event )
    {
        QLineEdit::resizeEvent ( event );
        updateButton();
    }

private:
    QPushButton *_button;
    QFileDialog *_fileDialog;
    void updateButton()
    {
        QRect buttonRect ( this->width()-31, 1, 30, this->height()-2 );
        _button->setGeometry ( buttonRect );
    }

};

}
#endif // PARAMSETWDGT_H
