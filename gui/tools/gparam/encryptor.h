#ifndef _ava_Encryptor_h
#define _ava_Encryptor_h
#include <iostream>
#include <string>
#include "gparam_exports.h"

/**Class for encripting and decripting messages
 */
class GPARAM_API Encryptor {
public:
  
    static void encrypt(std::istream &in,std::ostream &out,std::string key);
    static void decrypt(std::istream &in,std::ostream &out,std::string key);
    
    static void encrypt(std::string in,std::string &out,std::string key);
    static void decrypt(std::string in,std::string &out,std::string key);
};
  
#endif

    
