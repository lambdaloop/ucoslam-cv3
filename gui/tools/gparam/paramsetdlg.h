#ifndef PARAMSETDLG_H
#define PARAMSETDLG_H

 
#include "gparam.h"
#include "paramsetwdgt.h"
#include "gparam_exports.h"

namespace gparam
{
  
class GPARAM_API ParamSetDialog : public QDialog
{
    Q_OBJECT
public:
    explicit ParamSetDialog ( QWidget *parent=0, QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::Ok | QDialogButtonBox::Cancel,Qt::WindowFlags f = 0 );
    explicit ParamSetDialog ( const gparam::ParamSet &paramSet,QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::Ok | QDialogButtonBox::Cancel , QWidget *parent=0, Qt::WindowFlags f = 0 );
    explicit ParamSetDialog ( gparam::ParamSet *paramSet, QDialogButtonBox::StandardButtons buttons = QDialogButtonBox::Ok | QDialogButtonBox::Cancel ,QWidget *parent=0, Qt::WindowFlags f = 0 );
    ~ParamSetDialog();
    void setParamSet ( const  gparam::ParamSet &paramSet );
    void setParamSet ( gparam::ParamSet *paramSet );
    gparam::ParamSet getParamSet() const;
    void setButtons(QDialogButtonBox::StandardButtons buttons) { _paramSetWgt->setButtons(buttons);   };
    void setFileDialogPath(QString path){_paramSetWgt->setFileDialogPath(path);}

private:
    void createDialog(QDialogButtonBox::StandardButtons buttons);
    void applyChanges();
    ParamSetWdgt *_paramSetWgt;
    QVBoxLayout *_layout;
 signals:
    void paramChanged ( int idx );
    void okBtnPressed();
    void cancelBtnPressed();
    void applyBtnPressed();
    void advancedParamsToogled( bool );

private slots:
    void okPressed();
    void cancelPressed();
    void applyPressed(); 
    void paramSetModified(int);
    void on_advancedParamsToogled( bool );

};


}
#endif
