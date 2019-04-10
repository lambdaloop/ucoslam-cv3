#include "appparams.h"
using namespace std;

void  AppParams::saveToSettings ( gparam::ParamSet &obj, std::string baseName ) {
    QSettings settings;
    std::stringstream sstr;
    sstr<<obj;
    std::string name=baseName+obj.getSignature();
    settings.setValue ( name.c_str(), sstr.str().c_str() );
}

bool  AppParams::readFromSettings ( gparam::ParamSet &obj, std::string baseName ,bool forceWriteFirstTime  ) {
    QSettings settings;
    gparam::ParamSet aux;
    bool res;
    std::string name=baseName+obj.getSignature();
    QVariant val=settings.value ( name.c_str() );
    if ( val.isValid() ) {
        std::stringstream sstr;
        sstr<<val.toString().toStdString();
        sstr>>aux;
        obj.merge ( aux,true );
        res=true;
    } else {
        if (forceWriteFirstTime){
            saveToSettings(obj,baseName);
            return true;
        }
        res= false;
    }
    return res;

}
