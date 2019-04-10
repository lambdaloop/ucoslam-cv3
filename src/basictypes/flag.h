#ifndef UcoSLAM_Flag
#define UcoSLAM_Flag
#include <cstdint>
//convenient class for manipulating flags
struct Flag{

    void reset(){v=0;}

    bool is(unsigned char flag)const{return v&flag;}

    void set(unsigned char flag,bool b){
        if (b) v|=flag;
        else   v&=~ ( flag);
    }
     operator uint8_t() const{return v;}
    uint8_t v=0;
};
#endif
