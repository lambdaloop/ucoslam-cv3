#ifndef _Generic_Params_
#define _Generic_Params_
#include <string>
#include <iostream>
#include <sstream>
#include <streambuf>
#include <vector>
#include <map>
#include "gparam_exports.h"
#include "encryptor.h"
#include <exception>


namespace gparam
{

//%p full path to the file
//%d dir to the file
//%s file base name QFileInfo(%p).baseName()
//%a applicationDirPath
//Example: open file test_file.eje in dir /home/me/project
//%p: /home/me/project/test_file.eje
//%d: /home/me/project
//%s: test_file
//%a: <path to the installed location of this app>


class GPARAM_API Param
{
 public:
    enum ParamType {UNKNOWN,STRING,STRINGLIST,INTEGER,REAL,BOOLEAN,FILEPATH,EXISTING_FILEPATH,DIRPATH};
    //returns a string identifying the param type
    std::string getParamTypeStr ( ParamType ) const;

public:
    //creates empty param (_type=NONE)
    Param () ;
    //creates indicated param
    Param ( std::string name ,ParamType pt ) ;
    Param ( std::string name ) ;
    explicit Param ( std::string name, std::string val ) ;
    explicit Param ( std::string name,const std::vector<std::string> &strlist , int defaultElementList=-1) ;
    explicit Param ( std::string name, char* val ) ;
    explicit Param ( std::string name, int val ,int min=0,int max=0,int stepInc=0 ) ;
    explicit Param ( std::string name, double val ,double min=0,double max=0,double stepInc=0 ) ;
    explicit Param ( std::string name, bool val ) ;


 
    /** Returns the value. UNSAFE!!!! (can't return strings with spaces)
     */
    template<typename T> T  get() const ; //unsafe

    /**Returns an integer value
     */
    int asInt() const
    {
        return get<int>();
    }
    /**Returns a double value
     */
    double asDouble() const
    {
        return get<double>();
    }

    /**Returns a double value
     */
    std::string asString() const
    {
        return _value;
    }

    /**Returns the list of possible values if param is of type  STRINGLIST
     */
    std::vector<std::string> &getStringList()
    {
        return _strlist;
    }
    
    /**In stringlists, returns the index in stringlist that  value  has. If value is not in stringlist, returns -1
     */
    int getStrListIndex();
    /**Sets the value only (does not change type)
     */
    template<typename T> void  set ( T &v )  ;
    /**Sets the value and  type. If
     */
    template<typename T> void  set ( T &v,ParamType type )  ;
    //sets the value to v and the type to INTEGER
    Param & operator= ( int v )  ;
    //sets the value to v and the type to REAL
    Param & operator= ( double v )  ;
    //sets the value to v and the type to BOOLEAN
    Param & operator= ( bool v )  ;
    //sets the value to v and the type is now changed
    Param & operator= (const std::string &v )  ;
    Param & operator= (const char *v )  ;

    //compares two elements (comparion only considers name and value)
    bool operator== ( const Param &p )const;
    //compares two elements (comparion only considers name and value)
    bool operator!= ( const Param &p )const;
    /**Indicates if the param has limits specified
     */
    bool hasLimits() const;

    /**Gets lower and upper limits values
     */
    template<typename T> T  getLowerLimit ( )  ;
    template<typename T> T  getUpperLimit ( )  ;

    /**
     * Indicates if the parameter has data.
     */
    bool empty() const;
    /**A param is invalid if has no name
     */
    bool isValid() const;
    /**Returns the name
     */
    std::string getName() const;
    /**Sets the name
    */
    void setName ( const std::string & name );
    /**Sets the description
     */
    void setDescription ( const std::string & des )
    {
        _description=des;
    }
    /**Sets the description
     */
    void setExtra (const std::string &extra )
    {
        _extra=extra;
    }    /**Returns the description
     */
    std::string getDescription() const;
    /**Returns extra info
     */
    std::string getExtra()const;
    /**Returns the type
     */
    ParamType getType() const;
    /**Returns the step increment if indicated. If not it will be 0
     */
    double getStepIncrement() const
    {
        return _stepIncrement;
    }
    /**
     * Short print string for debugging purpouses
     */
    std::string getPrintString() const;
    //Ostream operators
    friend GPARAM_API std::ostream & operator<< ( std::ostream &str,const Param &p );

    //Ostream operators
    friend GPARAM_API std::istream & operator>> ( std::istream &str,  Param &p ) ;

private:
    std::string _name,_description,_value,_extra;
    std::vector<std::string> _strlist;

    void initValues();
    ParamType _type;
    double _min,_max;
    double _stepIncrement;


 
};
/**
 *
 */
class GPARAM_API ParamSet: public std::vector<Param>
{
    std::string _name;//name of the set
    Param _dummy;
    std::string _FirstAdvanced;//if advanced params, indicates the index of the first one

public:

    ParamSet(){}
    ParamSet(const std::string&name){_name=name;}
    void setName ( const std::string &name )
    {
        _name=name;
    }
    std::string getName() const
    {
        return _name;
    }
    //finds the required param. If not found, returns an inValid object
    Param &find ( const Param &p );
    const Param &find ( const Param &p ) const;
    Param &find ( std::string name );
    const Param &find ( std::string name )const;
    int getIndexOf (const  std::string &name )const;

    Param & operator[] ( size_t i );
    const Param & operator[] ( size_t i ) const;
    Param & operator[] ( const Param &p );
    const Param & operator[] ( const Param &p ) const;
    Param & operator[] ( std::string name );
    const Param & operator[] ( std::string name ) const;
    //convenient insert method
    Param &  insert ( const Param &p,bool overwriteOldValue=false );

    //from this moment, all inserted params will be considered advanced
    void insertAdvanced();
    int getFirstAdvanced()const;
    // merge two ParamSets
    void merge ( const ParamSet &PS, bool overwriteRepeatedValues=false );

    //
    friend GPARAM_API std::ostream &operator<< ( std::ostream &str,const ParamSet & ps );
    //
    friend GPARAM_API std::istream &operator>> ( std::istream &str, ParamSet & ps ) ;


    /** Save this to a file.
     * If !merge, removes previous content
     * Else,  the file is parsed looking for another paramset with the same name. Then, the whole file is copied
     * kept except for the paramset that is replaced with the contents of this
     */
    void saveToFile ( std::string filePath, bool merge=false ) ;
    void saveToFileEncrypted ( std::string filePath, std::string key, bool merge=false ) ;
    void saveToStreamEncrypted ( std::ostream& str,std::string key ) ;

    void saveToBinStream ( std::ostream& str) ;
    //finds the param set of name specified in a file and set its contents in ps_out. If  found return true, else returns false
    /**Reads this from a file.
     * @param filePath where to look for the data
     * @param name of the paramset to be found. If name is empty (by default), the getName() member is invoked for obtaining the name. Therefore,
     * this function is directly empmloyable by inherited classes that setName. If name=="*", then the first instance is read. If name =="" and this->getName()=="", then
     * it is equivalent to name=="*"
     *
     *
     * Example:
     * \code
     * class MyP: public gparam::ParamSet{
     *
     *  public:
     * MyP(){
     * setName("MyP");
     *  insert(  gparam::Param ( "minDecLevel",int ( 0 ),0,std::numeric_limits<int>::max() ));
     *  insert(  gparam::Param ( "ligthThres",int ( 15 ),0,255));
     * }
     * };
     *
     * int main(){
     *  MyP mp;
     *  mp.readFromFile(pathToFile); //equivalent to mp.readFromFile(pathToFile,mp.getName()); and mp.readFromFile(pathToFile,"MyP");
     * }
     * \endcode
     *
     */
    bool readFromFile ( std::string filePath,std::string name="" ) ;
    bool readFromFileEncrypted ( std::string filePath,std::string key,std::string name="" ) ;
    bool readFromStreamEncrypted ( std::istream &str,std::string key,std::string name="" ) ;
    bool readFromBinStream ( std::istream& str) ;

    /**
     */
    std::string  getPrintString() const;
    
    //indicates if two sets are equal
    bool operator ==(const ParamSet &p);
    //indicates if two sets are equal
    bool operator !=(const ParamSet &p);
    /**Returns a signature depending on the name of the elements and their type, but not on their values.
     * It can be used to know if a paramset has changed its params, added new or changed its typename
     */
    std::string getSignature();
};

class HyperParamSet: public std::vector<gparam::ParamSet>
{
public:
    //reads the set of param sets found in a file
    HyperParamSet readFromFile ( std::string filePath ) ;

};

/**Returns the value
  */
template<typename T> T  Param::get() const
{
    int size=sizeof ( T );
    if ( _type==INTEGER && size!=4 ) throw std::runtime_error( "Parameter has can not be converted to INTEGER. TYPE="+getParamTypeStr ( _type ));
    if ( _type==REAL && ( size!=8 && size!=4 ) ) throw std::runtime_error (  "Parameter has can not be converted to REAL. TYPE="+getParamTypeStr ( _type )  );
    if ( _type==STRING || _type==STRINGLIST )
    {
        std::cerr<<"UNSAFE OPERATION get<std::string>() in STRING AND STRINGLIST types since can not return spaced elements "<<__FILE__<<" "<<__LINE__<<std::endl;
    }
    std::stringstream sstr ( _value.c_str() );
    T tval;
    sstr>>tval;
    return tval;
}
/**Sets the value
 */
template<typename T> void Param::set ( T &v )
{

    int size=sizeof ( v );
    if ( _type==INTEGER && size!=4 ) throw  std::runtime_error (  "Parameter has can not be converted to INTEGER. TYPE="+getParamTypeStr ( _type )  );
    if ( _type==REAL && ( size!=8 && size!=4 ) ) throw  std::runtime_error (    "Parameter has can not be converted to REAL. TYPE="+getParamTypeStr ( _type ) );

    if ( _type==INTEGER  || _type==REAL )
    {
        if ( hasLimits() )
        {
            if ( v<_min || v>_max )
                throw  std::runtime_error (   "Parameter <"+_name+"> value can not be assigned out of bounds"  );
        }
    }

    std::stringstream sstr;
    sstr<<v;
    _value=sstr.str();

}
/**Sets the value
 */
template<typename T> void Param::set ( T &v,ParamType t )
{
    _type=t;
    set ( v );
}

/**Gets lower and upper limits values
 */
template<typename T> T Param::getLowerLimit()
{
    return _min;
}

template<typename T> T Param::getUpperLimit()
{
    return _max;
}



};
#endif


