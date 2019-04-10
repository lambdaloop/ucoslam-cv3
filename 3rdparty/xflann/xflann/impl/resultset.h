#ifndef _XFLANN_RESULTSET_
#define _XFLANN_RESULTSET_

#include <vector>
#include <limits>
#include <cinttypes>
#include <iostream>
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
//    ResultInfo<T> *array;
    T *distances;
    int *indices;
    size_t array_size=0;
    int maxSize;
    T maxValue;
    bool radius_search=false;

public:
    ResultSet(int MaxSize,  T MaxV=-1){
        maxSize=MaxSize;

        //set value for radius search
        if (MaxV==-1) maxValue =std::numeric_limits<T>::max();//no radius search
        else {
            maxValue =MaxV;
            radius_search=true;
        }

    }


    void reset(T *dist_ptr,int*indices_ptr){
        array_size=0;
        distances=dist_ptr;
        indices=indices_ptr;
    }

    inline void swap(int a,int b){
        std::swap(distances[a],distances[b]);
        std::swap(indices[a],indices[b]);

    }

    inline void push (const  ResultInfo<T> &val)
    {
        if (maxValue<val.dist) return;//for radius search
        if (array_size>=size_t(maxSize) ) {
            //check if the maxium must be replaced by this
            if ( val.dist<distances[0]){
                swap(0,array_size-1);
                array_size--;
                if (array_size > 1)  up (0) ;
            }
            else return;
        }

        distances[array_size]=val.dist;
        indices[array_size]=val.index;
        if (array_size>0) down ( array_size) ;
        array_size++;
    }


    inline T worstDist()const{
        if (radius_search)return maxValue;//radius search
        else
            if (array_size!=size_t(maxSize))return std::numeric_limits<T>::max(); return distances[0];}
    inline T top()const{assert(array_size); return distances[0];}
    inline size_t size()const{return array_size;}
private:


    inline void  down ( size_t index)
    {
        if (index == 0) return  ;
        size_t parentIndex =(index - 1) / 2;
        if (distances[ parentIndex]< distances[ index] ) {
            swap (  index, parentIndex) ;
            down (parentIndex) ;
        }
    }


    inline void up (size_t index)
    {
        size_t leftIndex  = 2 * index + 1 ;//vl_heap_left_child (index) ;
        size_t rightIndex = 2 * index + 2;//vl_heap_right_child (index) ;

        /* no childer: stop */
        if (leftIndex >= array_size) return ;

        /* only left childer: easy */
        if (rightIndex >= array_size) {
            if ( distances [ index] <distances[leftIndex]) {
                 swap ( index, leftIndex) ;
            }
            return ;
        }

        /* both childern */
        {
            if ( distances[ rightIndex]< distances[  leftIndex]  ) {
                /* swap with left */
                if (distances [index]< distances[leftIndex] ) {
                     swap ( index ,  leftIndex) ;
                    up ( leftIndex) ;
                }
            } else {
                /* swap with right */
                if ( distances[ index]  < distances[rightIndex]) {
                     swap ( index, rightIndex) ;
                    up ( rightIndex) ;
                }
            }
        }
    }
};

}
}


#endif
