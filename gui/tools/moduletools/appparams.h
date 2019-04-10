#ifndef _AppPARAMS_H
#define _AppPARAMS_H
#include <string>
#include "gparam/gparam.h"
#include <QSettings>
#include "exports.h"
#include <iostream>
class APP_MODULESET_TOOLS_API AppParams {

    public:


    template<typename T>
    static bool  saveToSettings ( const T &obj, std::string name ) {
        //try to implement a sort of mutex amongs processes
        QSettings settings;
        std::stringstream sstr;
        sstr<<obj;
        QByteArray ba ( sstr.tellp(),0 );
        sstr.read ( ba.data(),sstr.tellp() );
        settings.setValue ( name.c_str(),  ba );
        return true;
    }

    template<typename T>
    static bool  readFromSettings ( T &obj, std::string name ) {
        bool res;
        try {
            QSettings settings;
            QVariant val=settings.value ( name.c_str() );
            if ( val.isValid() ) {
                QByteArray ba=val.toByteArray();
                std::stringstream sstr;
                sstr.write ( ba.constData(),ba.size() );
                sstr>>obj;
                res=true;
            } else res=false;

        } catch ( std::exception &ex ) {
            res=false;
            std::cerr<<"error exception:"<<ex.what()<<std::endl;
        }
        return res;
    }

    static bool  readFromSettings ( std::string &obj, std::string name ) {
        bool res;
        try {
            QSettings settings;
            QVariant val=settings.value ( name.c_str() );
            if ( val.isValid() ) {
                QByteArray ba=val.toByteArray();
                std::stringstream sstr;
                sstr.write ( ba.constData(),ba.size() );
                obj=sstr.str();
                res=true;
            } else res=false;

        } catch ( std::exception &ex ) {
            res=false;
            std::cerr<<"error exception:"<<ex.what()<<std::endl;
        }
        return res;
    }



    template<typename T>
    static bool  saveToSettings ( const std::string &obj, std::string name ) {
        //try to implement a sort of mutex amongs processes
        QSettings settings;
        QByteArray ba ( obj.size(),0 );
        memcpy(ba.data(),&obj[0],obj.size());
        settings.setValue ( name.c_str(),  ba );
        return true;
    }

  static  void  saveToSettings ( gparam::ParamSet &obj, std::string baseName="" ) ;

  static  bool   readFromSettings (gparam::ParamSet &obj, std::string baseName="" , bool forceWriteFirstTime=true);

};



#endif
