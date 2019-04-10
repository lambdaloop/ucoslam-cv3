#include "paramsetwdgt.h"
#include <QDialogButtonBox>
#include <iostream>
using namespace std;
namespace gparam
{
void ParamSetWdgt::createButtonBox ( QDialogButtonBox::StandardButtons buttons )
{


    //_buttonlayout->removeWidget(_buttonBox);

    if ( !buttons.testFlag ( QDialogButtonBox::NoButton ) )
    {

        _buttonBox = new QDialogButtonBox ( buttons );
        if ( buttons.testFlag ( QDialogButtonBox::Ok ) )
        {

            connect ( _buttonBox->button ( QDialogButtonBox::Ok ), SIGNAL ( clicked ( bool ) ), this, SLOT ( okPressed() ) ) ;
        }
        if ( buttons.testFlag ( QDialogButtonBox::Cancel ) )
        {

            connect ( _buttonBox->button ( QDialogButtonBox::Cancel ), SIGNAL ( clicked ( bool ) ), this, SLOT ( cancelPressed() ) ) ;
        }

        if ( buttons.testFlag ( QDialogButtonBox::Apply ) )
        {
            _buttonBox->button ( QDialogButtonBox::Apply )->setEnabled ( false );

            connect ( _buttonBox->button ( QDialogButtonBox::Apply ), SIGNAL ( clicked ( bool ) ), this, SLOT ( applyPressed() ) ) ;
        }
        connect ( this, SIGNAL ( paramChanged ( int ) ), this, SLOT ( paramSetModified() ) ) ;

        _buttonlayout->addWidget ( _buttonBox );
    }
}
ParamSetWdgt::ParamSetWdgt ( QWidget *parent, QDialogButtonBox::StandardButtons buttons ) :
    QWidget ( parent )
{

    _buttons=buttons;
    createInitialElements();

}


ParamSetWdgt::ParamSetWdgt ( const gparam::ParamSet &paramSet,QWidget *parent, QDialogButtonBox::StandardButtons buttons ) :
    QWidget ( parent )
{


    _buttons=buttons;
    createInitialElements();
    setParamSet ( paramSet );
}
ParamSetWdgt::ParamSetWdgt ( gparam::ParamSet *paramSet,QWidget *parent , QDialogButtonBox::StandardButtons buttons ) : QWidget ( parent )
{
     _buttons=buttons;
     createInitialElements();
     setParamSet ( paramSet );
}

void ParamSetWdgt::createInitialElements(){

    _bLayout=new  QVBoxLayout();
    _buttonlayout=new QFormLayout();
    setLayout ( _bLayout );
    createButtonBox ( _buttons );
   advancedCheckBox=new QCheckBox();
    advancedCheckBox->setText(tr("Show Advanced"));
    advancedCheckBox->setChecked(false);
    connect(advancedCheckBox,&QCheckBox::toggled,this,&ParamSetWdgt::advancedToggled);
}
ParamSetWdgt::~ParamSetWdgt()
{
if (_buttonBox)  delete _buttonBox;
clear();
}



void ParamSetWdgt::    createLayouts(){

    _normalcontrols=new QWidget();
    _normalcontrols->setLayout(new QFormLayout());
    _extracontrols=new QWidget();
    _extracontrols->setLayout(new QFormLayout());


    _bLayout->addWidget(_normalcontrols);
    _bLayout->addWidget(advancedCheckBox);
    _bLayout->addWidget(_extracontrols);
    _bLayout->addLayout(_buttonlayout);
    _bLayout->addStretch();

}

void ParamSetWdgt::clear(){
    QLayoutItem *item;
    while ((item = _bLayout->takeAt(0)) != 0)
        _bLayout->removeItem (item);
    delete _normalcontrols;
    delete _extracontrols;
    _normalcontrols=_extracontrols=nullptr;
}


void ParamSetWdgt::addElement(QFormLayout *layout,const Param &p,int row){
    switch ( p.getType() )
    {

    case gparam::Param::STRING:
        addString (layout,p,row);
        break;

    case gparam::Param::STRINGLIST:
        addStringList ( layout,p,row);
        break;

    case gparam::Param::INTEGER:
        addInteger (layout, p,row);
        break;

    case gparam::Param::REAL:
        addReal (layout,p ,row);
        break;

    case gparam::Param::BOOLEAN:
        addBoolean (layout, p  ,row);
        break;

    case gparam::Param::FILEPATH:
    case gparam::Param::EXISTING_FILEPATH:
    case gparam::Param::DIRPATH:
        addFilepath (layout, p ,row);
        break;

    case gparam::Param::UNKNOWN:
    default:
        ;

    }

}
void ParamSetWdgt::setParamSet ( gparam::ParamSet *paramSet )
{

    clear();
    createLayouts();
    _paramSet =  paramSet;
    setWindowTitle ( paramSet->getName().c_str() );
    for ( size_t i=0; i<_paramSet->size(); i++ )
    {

        if( i>=_paramSet->getFirstAdvanced()  &&_paramSet->getFirstAdvanced()!=-1 )
            addElement((QFormLayout*)_extracontrols->layout(),_paramSet->at(i));
        else
            addElement((QFormLayout*)_normalcontrols->layout(),_paramSet->at(i));
    }
    if(_paramSet->getFirstAdvanced()!=-1)
        advancedCheckBox->show();
    else
        advancedCheckBox->hide();
    _extracontrols->hide();

}


void ParamSetWdgt::setParamSet ( const gparam::ParamSet & paramSet )
{

    internal_paramSet=paramSet;
    setParamSet ( &internal_paramSet );
}

void ParamSetWdgt::findSignalSender()
{    QObject* senderObj = QObject::sender();

     int row=_paramSet->getIndexOf(senderObj->objectName().toStdString());
      if(row!=-1){
          storeChange ( row, senderObj);
          emit paramChanged ( row );
      }

}


void ParamSetWdgt::storeChange ( int idx, QObject* obj )
{
    switch ( _paramSet->at ( idx ).getType() )
    {
    case gparam::Param::INTEGER :
    {
        QSpinBox* spinBox = static_cast<QSpinBox*> ( obj );
        _paramSet->at ( idx ) = int ( spinBox->value() );
        break;
    }

    case gparam::Param::REAL :
    {
        QDoubleSpinBox* doublespinBox = static_cast<QDoubleSpinBox*> ( obj );
        _paramSet->at ( idx ) = double ( doublespinBox->value() );
        break;
    }

    case gparam::Param::STRING :
    {
        QLineEdit* lineEdit = static_cast<QLineEdit*> ( obj );
        _paramSet->at ( idx ) = lineEdit->text().toStdString();
        break;
    }

    case gparam::Param::STRINGLIST :
    {
        QComboBox* comboBox = static_cast<QComboBox*> ( obj );
        _paramSet->at ( idx ) = comboBox->currentText().toStdString();
        break;
    }

    case gparam::Param::BOOLEAN :
    {
        QCheckBox* checkBox = static_cast<QCheckBox*> ( obj );
        _paramSet->at ( idx ) = checkBox->isChecked();
        break;
    }

    case gparam::Param::FILEPATH:
    case gparam::Param::EXISTING_FILEPATH:
    case gparam::Param::DIRPATH:
    {
        QFilePathLineEdit* filepathLineEdit = static_cast<QFilePathLineEdit*> ( obj );
        _paramSet->at ( idx ) = filepathLineEdit->text().toStdString();
        break;
    }

    case gparam::Param::UNKNOWN:
    default:
        ;

    }
}

void ParamSetWdgt::advancedToggled(bool checked)
{
    if(checked){
        _extracontrols->show();
        advancedCheckBox->setText(tr("Hide Advanced"));
    }
    else{
        advancedCheckBox->setText(tr("Show Advanced"));
        _extracontrols->hide();
    }
    adjustSize();

    emit advancedParamsToogled(checked);

}

void ParamSetWdgt::addAdvancedCheckBox(){
//    advancedCheckBox = new QCheckBox();
//    advancedCheckBox->setText(tr("Show Advanced"));
//    advancedCheckBox->setChecked(false);
//    connect(advancedCheckBox,&QCheckBox::toggled,this,&ParamSetWdgt::advancedToggled);
//    _layout->addRow ( tr(""), advancedCheckBox );
//   // elements.push_back(checkBox);

}

void ParamSetWdgt::addInteger ( QFormLayout *layout, gparam::Param intParam,int pos )
{
    QSpinBox* spinBox = new QSpinBox();
    spinBox->setToolTip ( QString::fromStdString ( intParam.getDescription() ) );
    spinBox->setWhatsThis ( QString::fromStdString ( intParam.getDescription() ) );
    if ( intParam.hasLimits() )
    {
        spinBox->setMinimum ( intParam.getLowerLimit<int>() );
        spinBox->setMaximum ( intParam.getUpperLimit<int>() );
    }
    else
    {
        spinBox->setMinimum ( std::numeric_limits<int>::min() );
        spinBox->setMaximum ( std::numeric_limits<int>::max() );
    }
    if ( intParam.getStepIncrement() !=0 )
        spinBox->setSingleStep ( intParam.getStepIncrement() );
    //set valuue must be after setting limits
    spinBox->setValue ( intParam.get<int>() );
    if(pos==-1)
        layout->addRow ( QString::fromStdString ( intParam.getName() ), spinBox );
    else
        layout->insertRow (pos, QString::fromStdString ( intParam.getName() ), spinBox );
    spinBox->setObjectName ( QString::fromStdString ( intParam.getName() ) );
    connect ( spinBox, SIGNAL ( valueChanged ( int ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(spinBox);
}

void ParamSetWdgt::addString ( QFormLayout *layout,gparam::Param strParam,int pos  )
{
    QLineEdit *lineEdit = new QLineEdit();
    lineEdit->setText ( QString::fromStdString ( strParam.asString() ) );
    lineEdit->setToolTip ( QString::fromStdString ( strParam.getDescription() ) );
    lineEdit->setWhatsThis ( QString::fromStdString ( strParam.getDescription() ) );
    if(pos==-1)
    layout->addRow ( QString::fromStdString ( strParam.getName() ), lineEdit );
    else
    layout->insertRow(pos, QString::fromStdString ( strParam.getName() ), lineEdit );
    lineEdit->setObjectName ( QString::fromStdString ( strParam.getName() ) );
    connect ( lineEdit, SIGNAL ( textChanged ( QString ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(lineEdit);
}

void ParamSetWdgt::addStringList (QFormLayout *layout, gparam::Param strlistParam ,int pos)
{
  //cout<<"ADD STRLIST="<<strlistParam<<endl;
    QComboBox* comboBox = new QComboBox();
    for ( size_t i=0; i<strlistParam.getStringList().size(); i++ )
        comboBox->addItem ( QString::fromStdString ( strlistParam.getStringList() [i] ) );
    
    comboBox->setCurrentIndex ( comboBox->findText ( QString::fromStdString ( strlistParam.asString() ) ) );
    comboBox->setToolTip ( QString::fromStdString ( strlistParam.getDescription() ) );
    comboBox->setWhatsThis ( QString::fromStdString ( strlistParam.getDescription() ) );
   if(pos==-1)
       layout->addRow ( QString::fromStdString ( strlistParam.getName() ), comboBox );
   else
       layout->insertRow ( pos,QString::fromStdString ( strlistParam.getName() ), comboBox );
    comboBox->setObjectName ( QString::fromStdString ( strlistParam.getName() ) );
    connect ( comboBox, SIGNAL ( currentIndexChanged ( int ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(comboBox);
}

void ParamSetWdgt::addReal (QFormLayout *layout, gparam::Param realParam,int pos )
{
    QDoubleSpinBox* doublespinBox = new QDoubleSpinBox();
    doublespinBox->setDecimals ( 4 );
    doublespinBox->setToolTip ( QString::fromStdString ( realParam.getDescription() ) );
    doublespinBox->setWhatsThis ( QString::fromStdString ( realParam.getDescription() ) );
    if ( realParam.hasLimits() )
    {
        doublespinBox->setMinimum ( realParam.getLowerLimit<double>() );
        doublespinBox->setMaximum ( realParam.getUpperLimit<double>() );
    }
    else
    {
        doublespinBox->setMinimum ( std::numeric_limits<int>::min() );
        doublespinBox->setMaximum ( std::numeric_limits<int>::max() );
    }
    //set value must be after setting limits
    doublespinBox->setValue ( realParam.get<double>() );
    if ( realParam.getStepIncrement() !=0 )
        doublespinBox->setSingleStep ( realParam.getStepIncrement() );

    if(pos==-1)
       layout->addRow ( QString::fromStdString ( realParam.getName() ), doublespinBox );
    else
        layout->insertRow ( pos,QString::fromStdString ( realParam.getName() ), doublespinBox );
    doublespinBox->setObjectName ( QString::fromStdString ( realParam.getName() ) );
    connect ( doublespinBox, SIGNAL ( valueChanged ( double ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(doublespinBox);
}

void ParamSetWdgt::addBoolean (QFormLayout *layout, gparam::Param boolParam,int pos  )
{
    QCheckBox* checkBox = new QCheckBox();
    checkBox->setChecked ( boolParam.get<bool>() );
    checkBox->setToolTip ( QString::fromStdString ( boolParam.getDescription() ) );
    checkBox->setWhatsThis ( QString::fromStdString ( boolParam.getDescription() ) );
    if(pos==-1)
        layout->addRow ( QString::fromStdString ( boolParam.getName() ), checkBox );
    else
    layout->insertRow ( pos,QString::fromStdString ( boolParam.getName() ), checkBox );
    checkBox->setObjectName ( QString::fromStdString ( boolParam.getName() ) );
    connect ( checkBox, SIGNAL ( toggled ( bool ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(checkBox);
}
void ParamSetWdgt::setFileDialogPath(QString path){
    fileDlgOpenPath=path;
    for(auto e:elements){
         QFilePathLineEdit *le=dynamic_cast<QFilePathLineEdit*>(e);
         if(le!=nullptr){
            le->getFileDialog()->setDirectory(fileDlgOpenPath);
         }
    }
}

void ParamSetWdgt::addFilepath (QFormLayout *layout, gparam::Param filepathParam , int pos )
{
    QFilePathLineEdit *filepathLineEdit = new QFilePathLineEdit();
    filepathLineEdit->setText ( QString::fromStdString ( filepathParam.asString() ) );
    if ( filepathParam.getType() ==gparam::Param::FILEPATH ) filepathLineEdit->getFileDialog()->setFileMode ( QFileDialog::AnyFile );
    else if ( filepathParam.getType() ==gparam::Param::EXISTING_FILEPATH ) filepathLineEdit->getFileDialog()->setFileMode ( QFileDialog::ExistingFile );
    else if ( filepathParam.getType() ==gparam::Param::DIRPATH ) filepathLineEdit->getFileDialog()->setFileMode ( QFileDialog::Directory );
    if(!fileDlgOpenPath.isEmpty())filepathLineEdit->getFileDialog()->setDirectory(fileDlgOpenPath);
    filepathLineEdit->setToolTip ( QString::fromStdString ( filepathParam.getDescription() ) );
    filepathLineEdit->setWhatsThis ( QString::fromStdString ( filepathParam.getDescription() ) );
    if(filepathParam.getDescription().find("*.")!=std::string::npos)
        filepathLineEdit->getFileDialog()->setNameFilter(filepathParam.getDescription().c_str());
    if(pos==-1)
    layout->addRow ( QString::fromStdString ( filepathParam.getName() ), filepathLineEdit );
    else
        layout->insertRow ( pos,QString::fromStdString ( filepathParam.getName() ), filepathLineEdit );
    filepathLineEdit->setObjectName ( QString::fromStdString ( filepathParam.getName() ) );
    connect ( filepathLineEdit, SIGNAL ( textChanged ( QString ) ), this, SLOT ( findSignalSender() ) );
    elements.push_back(filepathLineEdit);
}

void ParamSetWdgt::okPressed()
{
    emit okBtnPressed();
}
void ParamSetWdgt::cancelPressed()
{
    emit cancelBtnPressed();;
}
void ParamSetWdgt::applyPressed()
{
    emit applyBtnPressed();
    _buttonBox->button ( QDialogButtonBox::Apply )->setEnabled ( false );

}
void ParamSetWdgt::paramSetModified()
{
    if ( _buttonBox!=NULL )
    {
        QPushButton *applyButton=_buttonBox->button ( QDialogButtonBox::Apply );
        if(applyButton!=NULL) applyButton->setEnabled ( true );
    }
}


}














