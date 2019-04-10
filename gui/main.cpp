#include <QApplication>
#include <iostream>
#include <QSplashScreen>
#include <QCoreApplication>
#include <QMessageBox>
#include <QMainWindow>
#include <QSettings>
#include <QDir>
#include "mainwindow.h"

#ifdef WIN32
#include <windows.h>
void freeConsole() {
    FreeConsole();
}
#else
void freeConsole() {}
#endif

using namespace std;
class CmdLineParser{int argc; char **argv; public:
                    CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}
                                    bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    }
                                                    string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                   };


int main(int argc, char *argv[])
{


    CmdLineParser cml(argc,argv);

    if(!cml["-debug"])
       freeConsole();

    //check if the settings file is not created and create it
    QApplication QApp(argc, argv);
    QCoreApplication::setOrganizationName("UCOAVA");
    QCoreApplication::setApplicationName("UCOSLAM_GUI");


    if(cml["-clearsettings"]){
        QSettings settings;
        settings.remove("");
        exit(0);
    }
    QApp.processEvents();
    //go to application filepath to ensure the settings file can be properly readed

    QDir::setCurrent(QCoreApplication::applicationDirPath());

    MainWindow *mw=new MainWindow();

     QApp.processEvents();

    //now, go to user space
    QDir::setCurrent(QDir::homePath());
    mw->showMaximized();
    return QApp.exec();

}
