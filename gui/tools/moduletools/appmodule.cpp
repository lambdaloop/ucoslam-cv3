#include "appmodule.h"
using namespace std;
AppModule::AppModule (ModuleSetMainWindow *parent):QObject((QObject*)parent) {
    _isActive=false;
    _dock=0;
    _toolbar=0;
    _cwidget=0;
    _menu=0;
}

