#include "gparam.h"
#include <fstream>
#include <cstring>
#include <tuple>
#include <algorithm>
#include <stdint.h>
#include <cassert>
using namespace std;
namespace gparam
{

static void gparam_Param_toStream ( std::ostream &str,const std::string &strg,std::string name )
{
    str<<"<"<<name<<"> ";
    str<<strg.size() <<" ";
    str.write( strg.c_str(),strg.size()*sizeof(char));
    str<<" </"<<name<<"> ";
}
static void gparam_Param_toStream ( std::ostream &str,const std::vector<std::string> &strl ,std::string name )
{
    str<<"<"<<name<<"> ";
    str<<strl.size() <<" ";
    for ( size_t i=0; i<strl.size(); i++ ) gparam_Param_toStream ( str,strl[i],"elem" );
    str<<" </"<<name<<"> ";
}

static std::string gparam_Param_fromStream ( std::istream &str,std::string name )
{
    std::string strin;
    str>>strin;
//     cout<<"ss="<<strin<<endl;
    if ( strin.find ( name ) ==std::string::npos )
        throw std::runtime_error (  "param has not expected name ("+name+"!="+strin+")"  );
    int size;
    str>>size;
//     cout<<"size="<<size<<endl;
    std::string strout;
    strout.resize ( size );
    str.get();//burn one

    for ( int i=0; i<size; i++ ) strout[i]=str.get();
    //finally, read last string

//     cout<<"out="<<strout<<endl;
    str>>strin;
//     cout<<"ssend="<<strin<<endl;
    if ( strin.find ( name ) ==std::string::npos )
        throw std::runtime_error( "eof not expected "+strin );
    return strout;
}


static std::vector<std::string>   gparam_Param_fromStream_strlist ( std::istream &str,std::string name )
{
    std::string strin;
    str>>strin;
//     cout<<"gparam_Param_fromStream_strlist::"<<strin<<endl;
    if ( strin.find ( name ) ==std::string::npos )
        throw std::runtime_error("param has not expected name ("+name+"!="+strin+")" );

    int size;
    str>>size;
    std::vector<std::string> out ( size );
//     cout<<"size="<<size<<endl;

    for ( int i=0; i<size; i++ ) out[i]=gparam_Param_fromStream ( str,"elem" );
//     cout<<"gparam_Param_fromStream_strlist::"<<strin<<endl;
    str>>strin;
    if ( strin.find ( name ) ==std::string::npos )
        throw std::runtime_error (  "eof not expected "+strin );
    return out;
}
///---------------------------------------------------
/// IMPLEMENTATION
///---------------------------------------------------

void Param::initValues()
{
    _stepIncrement=0;
    _min=std::numeric_limits<float>::min();
    _max=std::numeric_limits<float>::max();
    _type=UNKNOWN;

}

Param::Param ()
{
    initValues();
}
/**
 */
Param::Param ( std::string name , Param::ParamType pt )
{
    initValues();
    _name=name;
    _type=pt;
    if ( _type==INTEGER || _type==REAL ) _value="-1";
    else if ( _type==BOOLEAN ) _value="0";
    else _value="";
}

/**
 */
Param ::Param ( std::string name )
{
    initValues();
    _name=name;
    _type=UNKNOWN;
}


/**
 */
Param::Param ( std::string name, char* val )
{
    initValues();
    _name=name;
    _value=val;
    _type=STRING;

}

//creates indicated param
Param::Param ( std::string name, std::string val )
{
    initValues();
    _name=name;
    _value=val;
    _type=STRING;
}

Param::Param ( std::string name,int val,int minv,int maxv,int stepInc )
{
    initValues();
    _name=name;
    _min=std::min ( minv,maxv );
    _max=std::max ( minv,maxv );
    set<int> ( val ,INTEGER );
    _stepIncrement=stepInc;

}
Param::Param (std::string name, const std::vector<string> &strlist , int defaultElementList) {
     initValues();
     _name=name;
     _strlist=strlist;
     _type=STRINGLIST ;
     if (defaultElementList>=0 && defaultElementList<int(_strlist.size()))
         _value=_strlist[defaultElementList];

}

Param::Param ( std::string name, double val,double minv ,double maxv ,double stepInc )
{
    initValues();
    _name=name;
    _min=std::min ( minv,maxv );
    _max=std::max ( minv,maxv );
    set<double> ( val,REAL );
    _stepIncrement=stepInc;

}


Param::Param ( std::string name, bool val )
{
    initValues();
    _name=name;
    _type=BOOLEAN;
    set<bool> ( val );
    _stepIncrement=_max=_min=0;

}


bool Param::operator== ( const Param &p )const
{
    return _value==p._value && _name==p._name;
}

bool Param::operator!= ( const Param &p )const
{
    return _value!=p._value || _name!=p._name;
}

Param & Param::operator= ( int v )
{
    set<int> ( v );
    return *this;
}
Param & Param::operator= ( double v )
{
    set<double> ( v );
    return *this;

}
Param & Param::operator= ( bool v )
{
    set<bool> ( v );
    return *this;

}
Param & Param::operator= ( const std::string &v )
{
    _value=v;
    return *this;

}
Param & Param::operator= (const char *v )  {
    _value=v;
    return *this;
}

/**
 */
int Param::getStrListIndex()
{

    if ( _value.empty() ) return -1;
    for ( size_t i=0; i<_strlist.size(); i++ )
        if ( _value==_strlist[i] ) return i;
    return -1;
}


/**
 *
 */
std::string Param::getParamTypeStr ( Param::ParamType t ) const
{
    switch ( t )
    {

    case UNKNOWN:
        return "UNKNOWN";
    case INTEGER:
        return "INTEGER";
    case REAL:
        return "REAL";
    case BOOLEAN:
        return "BOOLEAN";
    case STRINGLIST:
        return "STRINGLIST";
    case FILEPATH:
        return "FILEPATH";
    case EXISTING_FILEPATH:
        return "EXISTING_FILEPATH";
    case DIRPATH:
        return "DIRPATH";
    case STRING:
        return "STRING";
    default:
        return "?";
    }
}

std::string Param::getPrintString() const
{
    stringstream sstr;
    sstr<<"#Name="<<_name<<" #type="<<getParamTypeStr ( _type ) <<" #Val="<<_value<<" #range ["<<_min<<","<<_max<<"] #desc="<<_description<<" ";
    return sstr.str();
}

/**Indicates if the param has limits specified
    */
bool Param::hasLimits() const
{
    return _min!=_max;
}

/**
 * Indicates if the parameter has data.
 */
bool Param::empty() const
{
    return  _value.empty();
}
/**A param is invalid if has no name
 */
bool Param::isValid() const
{
    return !_name.empty();
}
/**Returns the name
 */
std::string Param::getName() const
{
    return _name;
}
/**Sets the name
*/
void Param::setName (const std::string &name )
{
    _name=name;
}
/**Returns the description
 */
std::string Param::getDescription() const
{
    return _description;
}
/**Returns the extra info
 */
std::string Param::getExtra() const
{
    return _extra;
}
/**Returns the type
 */
Param::ParamType Param::getType() const
{
    //creates empty param (_type=NONE)

    return _type;
}

//Ostream operators
std::ostream & operator<< ( std::ostream &str,const Param &p )
{
    str<<"<gparam::Param>\t";
    str<< p._type<< " "<<p._min<< " "<<p._max<<" "<<p._stepIncrement<<"\n\t";
    gparam_Param_toStream ( str,p._name,"name" );
    str<<"\n\t";
    gparam_Param_toStream ( str,p._description,"description" );
    str<<"\n\t";
    gparam_Param_toStream ( str,p._value,"value" );
    str<<"\n\t";
    gparam_Param_toStream ( str,p._strlist,"strlist" );
    str<<"\n\t";
    gparam_Param_toStream ( str,p._extra,"_extra" );
    str<<"\n";
    str<<"</gparam::Param>";
    return str;
}

//Ostream operators
std::istream & operator>> ( std::istream &str,  Param &p )
{
//     cout<<"operator>> ( std::istream &str,  Param &p )"<<endl;
    std::string strin;
    str>>strin;
    if ( strin!="<gparam::Param>" )
        throw std::runtime_error (  "<gparam::Param> not found"   );

    int t;
    str>>t;
    p._type= ( Param::ParamType ) t;
    str>> p._min>>p._max>>p._stepIncrement;
    p._name=gparam_Param_fromStream ( str,"name" );
    p._description=gparam_Param_fromStream ( str,"description" );
    p._value=gparam_Param_fromStream ( str,"value" );
    p._strlist=gparam_Param_fromStream_strlist ( str,"strlist" );

    auto pos=str.tellg();
    str>>strin;
    if ( strin.find("_extra")!=std::string::npos){
        str.seekg(pos);
        p._extra=gparam_Param_fromStream ( str,"_extra" );
        str>>strin;
    }

    if ( strin!="</gparam::Param>" )
        throw  std::runtime_error(  "</gparam::Param> not found"  );
    return str;
}


////////////////PARAMSET

//finds the required param

//finds the required param
Param &ParamSet::find ( const Param &p )
{
    _dummy=Param();
    for ( size_t i=0; i<size(); i++ )

        if ( p.getName() ==at ( i ).getName() ) return at ( i );
    //else , return dummy
    return _dummy;
}
const Param &ParamSet::find ( const Param &p ) const
{

    for ( size_t i=0; i<size(); i++ )

        if ( p.getName() ==at ( i ).getName() ) return at ( i );
    //else , return dummy
    return _dummy;

}

Param &ParamSet::find ( string name )
{
    return find ( Param ( name ) );
}
int ParamSet::getIndexOf(const  std::string &name )const{
    for ( size_t i=0; i<size(); i++ )
        if ( name==at ( i ).getName() ) return i;
    return -1;
}
const Param &ParamSet::find ( string name ) const
{
    return find ( Param ( name ) );
}
Param & ParamSet::operator[] ( size_t i )
{
    return at ( i );
}
const Param & ParamSet::operator[] ( size_t i ) const
{
    return at ( i );
}
Param & ParamSet::operator[] ( const Param &p )
{
    return find ( p );
}
const Param & ParamSet::operator[] ( const Param &p ) const
{
    return find ( p );
}
Param & ParamSet::operator[] ( std::string name )
{
    return find ( Param ( name ) );
}
const Param & ParamSet::operator[] ( std::string name ) const
{
    return find ( Param ( name ) );
}
//convenient insert method
Param &  ParamSet::insert ( const Param &p,bool overwriteOldValue )
{
    Param &pin=find ( p );
    if ( !pin.isValid() )
    {
        push_back ( p );
        return back();
    }
    else
    {
        if ( overwriteOldValue )	pin=p;
        return pin;
    }
}

// merge two ParamSets, repeated params are overwritten
void ParamSet::merge ( const ParamSet &PS, bool overwriteRepeatedValues )
{
    for ( size_t i=0; i<PS.size(); i++ ) insert ( PS[i], overwriteRepeatedValues );
}


/**
 */
std::string ParamSet::getPrintString() const
{
    stringstream sstr;
    sstr<<"@Param Set Name="<<_name<<endl;
    for ( size_t i=0; i<size(); i++ )
        sstr<<"\t"<<at ( i ).getPrintString() <<endl;
    return sstr.str();
}

//
std::ostream &operator<< ( std::ostream &str,const ParamSet & ps )
{
    str<<"<gparam::ParamSet>"<<std::endl;
    str<<"\t"<<0<<std::endl;
    str<<"\t"<<ps.size() <<" ";
    gparam_Param_toStream ( str,ps._name,"name" );
    str<<std::endl;

    for ( size_t i=0; i<ps.size(); i++ )
        str<<ps[i]<<std::endl;
    str<<"firstAdvanced "<< ps._FirstAdvanced.size()<<" ";
    str.write(&ps._FirstAdvanced[0],ps._FirstAdvanced.size());
    str<<" ";
    str<<"</gparam::ParamSet> ";
    return str;
}
//
std::istream &operator>> ( std::istream &str, ParamSet & ps )
{
    std::string strin;
    str>>strin;
    if ( strin!="<gparam::ParamSet>" )
        throw  std::runtime_error ( "<gparam::ParamSet> not found"  );
    int _encrypted;
    str>>_encrypted;
    if ( _encrypted )
        throw   std::runtime_error (  "Trying to read encrypted param set"  );
    int _size;
    str>>_size;
    ps.resize ( _size );
    ps._name=gparam_Param_fromStream ( str,"name" );


    for ( size_t i=0; i<ps.size(); i++ )
    {
//         cout<<"i="<<i<<endl;
        str>>ps[i];
    }
    str>>strin;

    if(strin.find("firstAdvanced")!=std::string::npos){
        int size;str>>size;
        ps._FirstAdvanced.resize(size);
        str.read(&ps._FirstAdvanced[0],ps._FirstAdvanced.size());
        str>>strin;
   }

    if ( strin!="</gparam::ParamSet>" )
        throw  std::runtime_error (  "</gparam::ParamSet> not found"  );

    return str;
}

struct ParamSetInfoFile
{

    ParamSetInfoFile()
    {
        start=end=0;
        encrypted=false;
    }
    bool isValid() const
    {
        return start!=end ;
    }
    streampos start,end;
    ParamSet ps;
    bool encrypted;
    string encryptedData;
    string name;
};

vector<ParamSetInfoFile > readAllParamSets ( istream &str )
{
    vector<ParamSetInfoFile > psi;
//look for character <


    while ( !str.eof() )
    {
        if ( str.get() =='<' )
        {
            //read next 17 characters gparam::ParamSet>
            char sig[18];
            str.read ( sig,17 );
            sig[17]='\0';
            if ( std::strcmp ( sig,"gparam::ParamSet>" ) ==0 )
            {
                str.seekg ( -18,ios::cur );
                //read the paramset
                psi.push_back ( ParamSetInfoFile() );
                psi.back().start=str.tellg();
                str>>psi.back().ps;
                psi.back().end=str.tellg();
            }
        }
    }
    return psi;
}



ParamSetInfoFile  findParamSet ( istream &str,std::string name )
{
//look for character <


    while ( !str.eof() )
    {
        if ( str.get() =='<' )
        {
            //read next 17 characters gparam::ParamSet>
            char sig[20];
            str.read ( sig,17 );
            sig[17]='\0';
            ParamSetInfoFile psi;
            if ( std::strcmp ( sig,"gparam::ParamSet>" ) ==0 )
            {
                str.seekg ( -18,ios::cur );
                psi.start=str.tellg();

                // check if it is encrypted or not
                str.read ( sig,18 );
                int _encrypted;
                str>>_encrypted;

                if ( !_encrypted )
                {
                    str.seekg ( psi.start ); // go back
                    str>>psi.ps; // read the whole param set
                    psi.end=str.tellg();
                    psi.encrypted = false;
                    psi.name = psi.ps.getName();
                }
                else
                {
                    int _nbytes;
                    str>>_nbytes;
                    psi.name=gparam_Param_fromStream ( str,"name" );
                    str.get(); // remove blank space
                    char *tmpStr = new char[_nbytes];
                    str.read ( tmpStr, _nbytes );
                    psi.encryptedData = std::string ( tmpStr, _nbytes );
                    delete[] tmpStr;
                    str.read ( sig,19 );
                    sig[19]='\0';
                    if ( std::strcmp ( sig,"</gparam::ParamSet>" ) !=0 ) ;//invalid end
                    psi.end=str.tellg();
                    psi.encrypted = true;
                }
            }
            if ( name.empty() || name=="*" ) return psi;
            else if ( psi.name ==name ) return psi;
        }
    }
    return ParamSetInfoFile();
}


void writeChunks ( istream & in, ostream &out,streampos posStart,streampos posEnd )
{
//   cout<<"WRITE CHUNK="<<posStart<<" "<<posEnd<<endl;
    char cad[1024];
    streampos toWrite=posEnd-posStart;
    in.seekg ( posStart,ios::beg );
    while ( toWrite>0 )
    {
        streampos toWrite_now=std::min ( streampos ( 1024 ),toWrite );
        in.read ( cad,toWrite_now );
        out.write ( cad,toWrite_now );
        toWrite-=toWrite_now;
    }
}

void  dumpAllBut ( istream &istr,ostream &ostr,streampos posStart,streampos posEnd )
{

    istr.seekg ( 0,ios::end ); //go to start
    streampos fsize=istr.tellg();

    if ( posStart==streampos ( 0 ) && fsize==posEnd ) return; //exclude all

    if ( posStart!=streampos ( 0 ) )
        writeChunks ( istr,ostr,0,posStart );
    if ( posEnd!=fsize )
        writeChunks ( istr,ostr,posEnd,fsize );
}

gparam::HyperParamSet  HyperParamSet ::readFromFile ( std::string filePath )
{
    ifstream ifile ( filePath.c_str(),ios::binary );
    if ( !ifile ) throw  std::runtime_error (  "File <"+filePath+"> not found"  );
    vector<ParamSetInfoFile> psi=readAllParamSets ( ifile );
    HyperParamSet  ret;
    for ( size_t i=0; i<ret.size(); i++ )
        ret.push_back ( psi[i].ps );
    return ret;
}

/**
 *
 */
bool ParamSet::readFromFile ( std::string filePath,std::string name )
{

    cerr<<"readFromFile 1 :"<< filePath.c_str()<<endl;
    ifstream ifile ( filePath.c_str(),ios::binary );
    if ( !ifile ) {
        cerr<<"readFromFile throw :"<< filePath.c_str()<<endl;
        throw  std::runtime_error(  "File <"+filePath+"> not found"  );
    }
    string findName;
    cerr<<"prm 1"<<endl;
    if ( name.empty() ) findName=getName();
    else findName=name;
    cerr<<"prm 2"<<endl;
    ParamSetInfoFile  psi=findParamSet ( ifile,findName );
    cerr<<"prm 3"<<endl;
    if ( !psi.isValid() ) return false;
    ( *this ) =psi.ps;
    cerr<<"prm 4"<<endl;

    return true;
}

bool ParamSet::readFromFileEncrypted ( string filePath, string key, string name )
{
    ifstream ifile ( filePath.c_str(),ios::binary );
    if ( !ifile ) throw  std::runtime_error(  "File <"+filePath+"> not found"  );
    return readFromStreamEncrypted ( ifile,key,name );
}

bool ParamSet::readFromStreamEncrypted ( std::istream &str,std::string key,std::string name )
{
    string findName;
    if ( name.empty() ) findName=getName();
    else findName=name;
    ParamSetInfoFile  psi=findParamSet ( str,findName );
    if ( !psi.isValid() || !psi.encrypted ) return false;

    std::string decryptParamSet;
    Encryptor::decrypt ( psi.encryptedData, decryptParamSet, key );
    std::stringstream strstm;
    strstm << decryptParamSet;
    try
    {
        strstm >> ( *this );
    }
    catch (std::exception &ex )
    {
        throw ex ;
    }
    return true;

}

/**
 *
 */
void ParamSet::saveToFile ( string filePath, bool merge )
{
    ofstream file;
    if ( merge )
    {
        ifstream ifile ( filePath.c_str(),ios::binary ); //open input
        if ( ifile )
        {
            stringstream tmpFile;      //create temporal
            ParamSetInfoFile   psi=findParamSet ( ifile, _name ); //find if this name is already

            if ( psi.isValid() ) //yes, save all but the i
            {
                dumpAllBut ( ifile,tmpFile, psi.start,psi.end );
                ifile.close();
                //now, reopen the file deleting content
                file.open ( filePath.c_str(),ios::binary );
                file<<tmpFile.rdbuf();//copy contents
            }
            else
                file.open ( filePath.c_str(),ios::binary|ios::app );
        }
    }

    if ( !file.is_open() ) file.open ( filePath.c_str(),ios::binary );
    if ( !file.is_open() ) throw  std::runtime_error ( "Could not open file:"+filePath  );
    //now, time to write our data
    file << ( *this );



}
    
    
    void ParamSet::saveToBinStream ( std::ostream& str) {
     std::ostringstream decryptParamSet;
    decryptParamSet<< ( *this );
    size_t si=decryptParamSet.str().size();
    str.write((char*)&si,sizeof(si));
    str.write((char*) decryptParamSet.str().c_str(),si);
    }
    bool ParamSet::readFromBinStream ( std::istream& str) {
    size_t si;
    str.read((char*)&si,sizeof(si));
    string inData;inData.resize(si);
    str.read((char*)&(inData[0]),si);
    stringstream sstr(inData);
    sstr>>(*this);
    return true;
    }

/**
 *
 */
void ParamSet::saveToStreamEncrypted ( std::ostream& str,std::string key )
{
    // encrypt
    std::ostringstream decryptParamSet;
    std::string encryptParamSet;
    decryptParamSet<< ( *this );
    Encryptor::encrypt ( decryptParamSet.str(), encryptParamSet, key );

    // save to file
    str << "<gparam::ParamSet>"<<std::endl;
    str << "\t" << 1 << std::endl;
    str << "\t" << encryptParamSet.size() << std::endl;
    gparam_Param_toStream ( str , getName() ,"name" );
    str << encryptParamSet;
    str << "</gparam::ParamSet>";
}

/**
 *
 */
void ParamSet::saveToFileEncrypted ( string filePath, string key, bool merge )
{
    ofstream file;
    if ( merge )
    {
        ifstream ifile ( filePath.c_str(),ios::binary ); //open input
        if ( ifile )
        {
            stringstream tmpFile;      //create temporal
            ParamSetInfoFile   psi=findParamSet ( ifile, _name ); //find if this name is already

            if ( psi.isValid() ) //yes, save all but the i
            {
                dumpAllBut ( ifile,tmpFile, psi.start,psi.end );
                ifile.close();
                //now, reopen the file deleting content
                file.open ( filePath.c_str(),ios::binary );
                file<<tmpFile.rdbuf();//copy contents
            }
            else
                file.open ( filePath.c_str(),ios::binary|ios::app );
        }
    }

    if ( !file.is_open() ) file.open ( filePath.c_str(),ios::binary );
    if ( !file.is_open() ) throw  std::runtime_error( "Could not open file:"+filePath  );
    //now, time to write our data
    saveToStreamEncrypted ( file,key );

}

uint64_t  getStrSignature ( string str )
{

    uint64_t res=0;
    for ( size_t i=0; i<str.size(); i++ )
        res+=str[i];
    return res;
}

uint64_t _getSignature(gparam::Param &p)
{
    uint64_t sig=0;
    sig+=p.getType()+getStrSignature ( p.getName())+getStrSignature(p.getDescription());
    //create a sig for each elemt y string list
    for ( size_t i=0; i<p.getStringList().size(); i++ ) sig+=getStrSignature ( p.getStringList()[i] );

    return sig;
}

std::string ParamSet::getSignature()
{
    stringstream ssig;
    ssig<<getName();
    uint64_t sign=getFirstAdvanced()>=0?1:0;
    for ( size_t i=0; i<size(); i++ )
        sign+=_getSignature(at(i));
    ssig<<"_"<<sign;
    return ssig.str();
    }
  bool ParamSet::operator ==(const ParamSet &p){
      bool equal=true;
      for(size_t i=0;i<p.size() && equal;i++)
      {
	equal&= (p[i]==at(i));
      }
      return equal;
    }
    

  bool ParamSet::operator !=(const ParamSet &p){
    return !(p==*this);
    }
    

  void ParamSet::insertAdvanced(){
      if (size()>0)
          _FirstAdvanced=back().getName();
  }

  int ParamSet::getFirstAdvanced()const{
      if(_FirstAdvanced.empty())return -1;
      for(size_t i=0;i<size();i++)
          if( at(i).getName()==_FirstAdvanced) {
              if( i==size()-1) return -1;
              else return i+1;
          }
      return -1;
  }

}
  
