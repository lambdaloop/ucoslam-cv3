#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "moduletools/modulesetmainwindow.h"



class MainWindow : public ModuleSetMainWindow {
    Q_OBJECT

public:
    explicit MainWindow ( QWidget *parent = 0  );

public slots:
    void on_global_action(const gparam::ParamSet &paramset);


};

#endif // MAINWINDOW_H
