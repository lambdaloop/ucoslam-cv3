#include "modulesetmainwindow.h"
#include "progresswindow.h"
#include <iostream>
#include <QTimer>
#include <QStatusBar>
#include <cmath>
using namespace std;
ModuleSetMainWindow::ModuleSetMainWindow(QWidget *parent) :
    QMainWindow(parent)
{
    qRegisterMetaType<std::string>("std::string");
    _stckWidget=new QStackedWidget ( this );
    setCentralWidget ( _stckWidget );

    _modules_tb=new QToolBar("modules",this);
    _modules_tb->setMovable(false);
    _modules_tb->setFloatable(false);
    _modules_tb->setOrientation( Qt::Vertical);
    _modules_tb->setIconSize(QSize(48,48));
    _modules_tb->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    addToolBar(Qt::LeftToolBarArea, _modules_tb  );

    _modules_actiongroup=new QActionGroup(this);
    QObject::connect(_modules_actiongroup,SIGNAL(triggered(QAction*)),this,SLOT(on_module_activated(QAction*)));

}

ModuleSetMainWindow::~ModuleSetMainWindow()
{

}

bool ModuleSetMainWindow::activateModule(std::string  name){
    if (moduleMap.find(name)==moduleMap.end())
        return false;
    moduleMap[name].action->trigger();
    _cur_active_module=name;
    return true;
}

void ModuleSetMainWindow::addModule ( std::string name, std::shared_ptr<AppModule> module ) {
    int _module_centralwidget_idx=-1;

    if ( module->getCentralWidget() ==0 ) {
        std::cerr<<"No central widget for module"<<module->getName() <<endl;
        exit(0);
    }

    _stckWidget->addWidget ( module->getCentralWidget() );
    std::cerr<<"_tsdftoolbox_module->getName()"<<module->getName() <<" "<<_stckWidget->count() <<endl;;
    _module_centralwidget_idx=_stckWidget->count()-1;
    QObject::connect( module.get(),&AppModule::notify_action_progress,
                      this,&ModuleSetMainWindow::on_progress_message_emitted);

    if ( module->getToolBar() !=0 ) {
        addToolBar ( module->getToolBar() );
    }
    if ( module->getControlPanel() !=0 ) {
        addDockWidget ( Qt::RightDockWidgetArea, module->getControlPanel() );
        module->getControlPanel()->setFeatures(QDockWidget::NoDockWidgetFeatures);

        //        module->getControlPanel()->set
       module->getControlPanel() ->setMinimumWidth ( 150 );
       module->getControlPanel() ->adjustSize();
    }


    module->deactivate();
    //now, add connections between this and the rest of modules
    for(auto mod:moduleMap){
        QObject::connect( module.get(),&AppModule::global_action_triggered,mod.second.get().get(),&AppModule::on_global_action);
        QObject::connect( mod.second.get().get(),&AppModule::global_action_triggered,module.get(),&AppModule::on_global_action);
    }


    connect( module.get(),&AppModule::global_action_triggered,this,&ModuleSetMainWindow::on_global_action);
    connect( this,&ModuleSetMainWindow::global_action_triggered,module.get(),&AppModule::on_global_action);
    //add to module map
    moduleMap.insert ( make_pair ( name, ModuleInfo ( module,_module_centralwidget_idx ) ) );
    moduleMap[name].action=_modules_tb->addAction(module->getIcon(),name.c_str());
    moduleMap[name].action->setCheckable(true);

    _modules_actiongroup->addAction(moduleMap[name].action);

}

void on_global_action_internal(const gparam::ParamSet &paramset);

void ModuleSetMainWindow::on_module_activated(QAction*action){
    std::string module_name=action->text().toStdString();
    _cur_active_module=module_name;
    //deactivate all but the selected
    for(std::unordered_map<std::string,ModuleInfo>::iterator m= moduleMap.begin();m!=moduleMap.end();m++)
        if (m->first !=module_name)
            m->second.get()->deactivate();
    //now, activate the one
    moduleMap[module_name].get()->activate();
    _stckWidget->setCurrentIndex (moduleMap[module_name].getIndex() );

    //call to inform
    on_module_activated(module_name,moduleMap[module_name]);



}

void ModuleSetMainWindow::on_progress_message_emitted (std::string action_name, int value,std::string message ,int minScaleRange,int maxScaleRange ){
    if ( value>=0 && minScaleRange<maxScaleRange && minScaleRange!=-1 && maxScaleRange!=-1 )
        value= ( maxScaleRange-minScaleRange ) * ( float ( value ) /100. ) + minScaleRange;
    std::cerr<<"action_name:"<<action_name<<" "<<value<<":"<<message<<endl;


    if (progressWindowsMaps.find(action_name)==progressWindowsMaps.end() && value==100) return;

    if (progressWindowsMaps.find(action_name)==progressWindowsMaps.end() && value!=100){
        progressWindowsMaps.insert( std::make_pair(action_name,std::make_shared<progresswindow>  (action_name)));
        statusBar()->addWidget(progressWindowsMaps[action_name].get());
    }

    progressWindowsMaps[action_name]->notify_action_progress(value,message);

    if (std::fabs(float(value))==100){
        progressWindowsMaps_toBeRemoved.push_back(progressWindowsMaps[action_name]);
        //wait a while and remove the widget
        progressWindowsMaps.erase( progressWindowsMaps.find( action_name));
        if (value<0)
            QTimer::singleShot(5000, this, SLOT(on_remove_progress_message()));
        else
            QTimer::singleShot(1000, this, SLOT(on_remove_progress_message()));
    }
}

void ModuleSetMainWindow::on_remove_progress_message(){
    for(size_t i=0;i<progressWindowsMaps_toBeRemoved.size();i++)
        statusBar()->removeWidget(progressWindowsMaps_toBeRemoved[i].get());
    progressWindowsMaps_toBeRemoved.clear();

}

bool ModuleSetMainWindow::isModule(string name){
            return moduleMap.find(name)!=moduleMap.end();
}

