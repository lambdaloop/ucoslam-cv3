#include "encryptor.h"
#include <cassert>
#include <vector>
#include <sstream>
#ifdef _MSC_VER
typedef __int32 int32_t;
typedef unsigned __int32 uint32_t;
#endif
using namespace std;
/* take 64 bits of data in v[0] and v[1] and 128 bits of key[0] - key[3] */

void encipher(uint32_t num_rounds, uint32_t v[2],const vector<uint32_t> & key) {
    assert(key.size()==4);
    uint32_t i;
    uint32_t v0=v[0], v1=v[1], sum=0, delta=0x9E3779B9;
    for (i=0; i < num_rounds; i++) {
        v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
        sum += delta;
        v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum>>11) & 3]);
    }
    v[0]=v0;
    v[1]=v1;
}

void decipher(uint32_t num_rounds, uint32_t v[2],const vector<uint32_t>  &key) {
    assert(key.size()==4);
    uint32_t i;
    uint32_t v0=v[0], v1=v[1], delta=0x9E3779B9, sum=delta*num_rounds;
    for (i=0; i < num_rounds; i++) {
        v1 -= (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + key[(sum>>11) & 3]);
        sum -= delta;
        v0 -= (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + key[sum & 3]);
    }
    v[0]=v0;
    v[1]=v1;
}

vector<uint32_t>   getStringKey(string skey){

uint32_t keyv[4]={0,0,0,0};
int maxSizeChar=4*sizeof(uint32_t);
char *ck=(char *)&keyv;
for(size_t i=0;i<skey.size();i++)
	ck[i%maxSizeChar]+=skey[i];
//ok, now set in the output
vector<uint32_t> outKey(4);
for(size_t i=0;i<4;i++)
	outKey[i]=keyv[i];
return outKey;
}

/**
 */
void Encryptor::encrypt(istream &in,ostream &out,string skey) {
    //read in blocks of 64 bits
    vector<uint32_t> key=getStringKey(skey);
    char  data[8]; 
    int totalSize=0;
    do {
	int readed=0;
	do{
	  in.read((char*)data+readed,sizeof(char));
	  if (!in.eof()) readed++;
	}while(readed<8 && !in.eof());
	
	totalSize+=readed;
	if (readed==8) encipher(skey[0],(uint32_t *)data,key);
	out.write(data,readed*sizeof(char));
	
    }while (!in.eof());
}

/**
 */
void Encryptor::decrypt(istream &in,ostream &out,string  skey) {
    //read in blocks of 64 bits
    vector<uint32_t> key=getStringKey(skey);
    char  data[8];
    int totalSize=0;
    do {
	int readed=0;
	do{
	  in.read((char*)data+readed,sizeof(char));
	  if (!in.eof()) readed++;
	}while(readed<8 && !in.eof());
	
	if (readed==8) decipher(skey[0],(uint32_t *)data,key);
	totalSize+=readed;
	out.write(data,readed*sizeof(char));
	
    }while (!in.eof());
}

/**
 */
void Encryptor::encrypt(std::string in,std::string &out,std::string key){
  std::stringstream sin(in),sout;
  encrypt(sin,sout,key);
  out=sout.str();
}
/**
 */
void Encryptor::decrypt(std::string in,std::string &out,std::string key){
  stringstream sin(in),sout;
  decrypt(sin,sout,key);
  out=sout.str();    
}


