#ifndef _XFLANN_RESULTSET_
#define _XFLANN_RESULTSET_

#include <vector>
#include <algorithm>
#include <limits>
namespace xflann{
namespace impl{
template<typename DType>
struct ResultInfo{
    ResultInfo(){}
    ResultInfo(DType Dist,uint32_t  Index=std::numeric_limits<uint32_t>::max()):dist(Dist),index(Index){}
    inline ResultInfo&operator=(DType d){dist=d;return *this;}
    inline bool operator<(const ResultInfo & dist_index) const
    {
        return (dist < dist_index.dist) || ((dist == dist_index.dist) && index < dist_index.index);
    }

    DType dist;
    uint32_t index=std::numeric_limits<uint32_t >::max();
    friend std::ostream &operator<<(std::ostream &str,const ResultInfo &r){str<<"("<<r.index<<","<<r.dist<<")";return str;}
};

//heap having at the top the maximum element
template<typename T>
class ResultSet{
public:
    std::vector<ResultInfo<T> >array;
    int maxSize;
    ResultInfo<T> maxValue;

public:
    ResultSet(int MaxSize,T MaxV=-1){
        maxSize=MaxSize;
        //set value for radius search
        if (MaxV==-1) maxValue.dist=std::numeric_limits<T>::max();//no radius search
        else maxValue.dist=MaxV;

    }

    ~ResultSet( ){
        //if (array!=0) delete array;
    }

    void reset(){ array.clear(); }

    inline void push (const  ResultInfo<T> &val)
    {
        if (maxValue<val) return;//for radius search

        if (array.size()>=maxSize ) {
            //check if the maxium must be replaced by this
            if ( val<array.back()) array.back()=val;
        }
        else array.push_back(val);
        std::sort(array.begin(),array.end());
    }


    inline size_t size() const{return array.size();}

};




}
}


#endif
