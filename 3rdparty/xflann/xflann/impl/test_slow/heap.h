#ifndef _XFLANN_HHEAP_
#define _XFLANN_HHEAP_
#include <vector>
#include <algorithm>
namespace xflann{
namespace impl{
//heap having at the top the minimum element
template<typename T,int maxSize=1000>
class Heap{
public:
    std::vector<T> array;

public:
    Heap(){}
    void reset(){ array.clear();}

    inline void push (const T &val)
    {
        array.push_back(val);
        std::sort(array.rbegin(),array.rend());
    }


    inline T pop ( ){
        T res=array.back();
        array.pop_back();
        return res;
    }

    inline size_t size()const{return array.size();}
    inline bool empty()const {return array.size()==0;}
private:



};


}
}


#endif
