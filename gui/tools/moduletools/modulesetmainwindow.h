#ifndef MODULESETMAINWINDOW_H
#define MODULESETMAINWINDOW_H

#include <QMainWindow>
#include <QStackedWidget>
#include <QActionGroup>
#include <unordered_map>
#include <string>
#include <memory>
#include "appmodule.h"
#include "progresswindow.h"
#include "exports.h"
struct APP_MODULESET_TOOLS_API  ModuleInfo{
    ModuleInfo(){}
    ModuleInfo(std::shared_ptr<AppModule> Module,int Centralwidget_idx){
        module=Module;
        centralwidget_idx=Centralwidget_idx;
    }
    std::shared_ptr<AppModule> operator()(){return module;}
    std::shared_ptr<AppModule> get(){return module;}
    template<typename T>
    T*  cast(){return  module->cast<T>();}
    int getIndex(){return centralwidget_idx;}


    std::shared_ptr<AppModule> module;
    int centralwidget_idx;
    QAction *action;
};

class APP_MODULESET_TOOLS_API ModuleSetMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit ModuleSetMainWindow(QWidget *parent = 0);
    ~ModuleSetMainWindow();
    void addModule ( std::string name, std::shared_ptr<AppModule> module ) ;
    std::unordered_map<std::string,  ModuleInfo >& getModuleMap(){return moduleMap;}
    std::string getActiveModule()const{return _cur_active_module;}
    bool activateModule(std::string );
    bool isModule(std::string);

public slots:
    virtual void on_global_action(const gparam::ParamSet &paramset){}
    virtual void on_module_activated(std::string moduleName,ModuleInfo minfo){}

private slots:
    void on_module_activated(QAction*);
    void on_progress_message_emitted (std::string action_name, int value,std::string message ,int minScaleRange=-1,int maxScaleRange=-1 );
    void on_remove_progress_message(  );

signals:
    void global_action_triggered(const gparam::ParamSet &paramset);
private:
    QStackedWidget *_stckWidget;
    QToolBar *_modules_tb;
    QActionGroup *_modules_actiongroup;
    std::unordered_map<std::string,  ModuleInfo > moduleMap;
    std::unordered_map<std::string,  std::shared_ptr<progresswindow> > progressWindowsMaps;
    std::vector<std::shared_ptr<progresswindow> > progressWindowsMaps_toBeRemoved;
    std::string _cur_active_module;


};

#endif // MODULESETMAINWINDOW_H
