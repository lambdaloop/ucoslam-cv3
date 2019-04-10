#include "mainwindow.h"
#include <iostream>
#include "modulecalibration/modulecalibration.h"
//#include "mapviewer/mapviewer.h"
#include "mappingmodule/mappingmodule.h"
using namespace std;





MainWindow::MainWindow ( QWidget *parent  ) :
ModuleSetMainWindow ( parent )  {
    try {
        setWindowIcon(QIcon(":/images/program_icon.ico"));
        setWindowTitle ( "UcoSLAM GUI" );
        addModule ( "Mapping",std::make_shared<MappingModule>(this));
//        addModule ( "Map Viewer", std::make_shared< MapViewer> (this) );
        addModule ( "Calibration", std::make_shared< ModuleCalibration> (this) );
        activateModule("Mapping");
    } catch ( std::exception &ex ) {
        cerr<<ex.what() <<endl;
    }



}

void MainWindow::on_global_action(const gparam::ParamSet &paramset){
cerr<<paramset<<endl;
}
