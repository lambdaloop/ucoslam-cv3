#ifndef APPMODULE_H_TOOLSET
#define APPMODULE_H_TOOLSET
#include <QObject>
#include <QWidget>
#include <QDockWidget>
#include <QToolBar>
#include <QIcon>
#include <QMenu>
#include <QBoxLayout>
#include <string>

#include "exports.h"
namespace gparam{
class ParamSet;
}
class ModuleSetMainWindow;

class APP_MODULESET_TOOLS_API  AppModule  :  public QObject {
    Q_OBJECT

public:
    AppModule (   ModuleSetMainWindow *parent=nullptr);
   QWidget *parent(){return (QWidget *)QObject::parent();}
    virtual ~AppModule() {}


    virtual std::string getName() const=0;
    virtual std::string getToolBoxTitle() const=0; //{return "";}

    template<typename T> T* getControlPanel_() {
        return dynamic_cast<T*> ( getControlPanel()->widget() );
    }
    QDockWidget *getControlPanel() {return _dock;}
    QWidget *getCentralWidget() {return  _cwidget;}
    QToolBar *getToolBar() {return _toolbar;}
    QIcon getIcon() {return _icon;}
    QMenu * getMenu() {return _menu;}


    template<typename T> T *cast() {
        return dynamic_cast<T*> ( this );
    }

    bool isActive()const{return _isActive;}

public slots:
    //call to activate
    void activate() {
        _isActive=true;
        if ( getToolBar() !=0 ) getToolBar()->show();
        if ( getControlPanel() !=0 ) getControlPanel()->show();

        on_activate();
        emit activated();
    }
    //call to deactivate
    void deactivate() {
        _isActive=false;
        if ( getToolBar() !=0 ) getToolBar()->hide();
        if ( getControlPanel() !=0 ) getControlPanel()->hide();
        on_deactivate();
        emit deactivated();
    }
public:
    //reimplement to act on activation/deactivation
    virtual void on_activate() {}
    virtual void on_deactivate(){}
    virtual void on_global_action(const gparam::ParamSet &paramset){}


signals:
    //
    void notify_action_progress(std::string action_name,int completeness,std::string message ,int minScaleRange=-1,int maxScaleRange=-1);
    void global_action_triggered(const gparam::ParamSet &paramset);//use it to inform the application of something

    /////////////////////////////////
    void activated();
    void deactivated();
protected:
    void setControlPanel ( QWidget *wdgt ) {
        if(_dock!=nullptr) delete _dock;
        _dock=new QDockWidget ( getToolBoxTitle().c_str() );
        QWidget *w=new QWidget;
        QBoxLayout *dockLayout = new QHBoxLayout;
        dockLayout->addSpacing(10);
        dockLayout->addWidget(wdgt);
        dockLayout->addSpacing(10);
        w->setLayout(dockLayout );
        _dock->setWidget ( w );

    }
    void setControlPanel ( QDockWidget *dock ) {
        _dock=  dock  ;
    }
    void setToolBar ( QToolBar *tb ) {
        _toolbar=tb;
    }
    void setCentralWidget ( QWidget *wgt ) {
        _cwidget=wgt;
    }
    void setIcon ( QIcon  icon ) {
        _icon=icon;
    }

    void setMenu ( QMenu *m ) {
        _menu=m;
    }
private:
    QDockWidget *_dock=nullptr;
    QToolBar *_toolbar=nullptr;
    QWidget *_cwidget=nullptr;
    QIcon _icon;
    QMenu *_menu=nullptr;
    bool _isActive;



};

#endif // APPMODULE_H
