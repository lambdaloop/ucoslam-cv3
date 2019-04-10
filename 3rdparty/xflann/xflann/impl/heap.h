#ifndef _XFLANN_HHEAP_
#define _XFLANN_HHEAP_
#include <cstdint>
#include <iostream>
namespace xflann{
namespace impl{
//heap having at the top the minimum element
template<typename T,int maxSize=5000>
class Heap{
  public:
    T array [maxSize];
    size_t array_size=0;

public:
    Heap(){}
    void reset(){
        array_size=0;
    }

    inline void push (const T &val)
    {
        if (array_size<maxSize ) {
            array[array_size]=val;
            if (array_size>0) down ( array_size) ;
            array_size++;
        }
        else std::cerr<<"Heap max size reached"<<std::endl;
    }


    inline T pop ( ){
        assert (array_size) ;
        T res=array[0];
        std::swap(array[0],array[array_size-1]);
        array_size--;
        if (array_size > 1)  up (0) ;
        return res;
    }

    inline size_t size()const{return array_size;}
    inline bool empty()const {return array_size==0;}
private:


    inline void
    down ( size_t index)
    {
        if (index == 0) return  ;

        size_t parentIndex =(index - 1) / 2;

        if (array[ index]< array[ parentIndex]) {
            std::swap (array[ index], array[parentIndex]) ;
            down (parentIndex) ;
        }
    }




    inline void up (size_t index)
    {
        size_t leftIndex  =   2 * index + 1;//vl_heap_left_child (index) ;
        size_t rightIndex = 2 * index + 2  ;// vl_heap_right_child (index) ;

        /* no childer: stop */
        if (leftIndex >= array_size) return ;

        /* only left childer: easy */
        if (rightIndex >= array_size) {
            if (array[leftIndex]< array [ index] ) {
                std::swap (array[index], array[leftIndex]) ;
            }
            return ;
        }

        /* both childern */
        {
            if ( array[  leftIndex] < array[ rightIndex]) {
                /* swap with left */
                if (array[leftIndex]< array [index] ) {
                    std::swap (array [index], array[leftIndex]) ;
                    up ( leftIndex) ;
                }
            } else {
                /* swap with right */
                if ( array[rightIndex]< array[ index]  ) {
                    std::swap (array [index], array[rightIndex]) ;
                    up ( rightIndex) ;
                }
            }
        }
    }


};



}
}


#endif
