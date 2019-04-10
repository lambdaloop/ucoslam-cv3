#include "ucoslam.h"

int main(int argc,char **argv){

    if(argc!=3) return -1;

    ucoslam::UcoSlam slam;
    slam.readFromFile(argv[1]);
    slam.getMap()->removeUnUsedKeyPoints();
    slam.getMap()->saveToFile(argv[2]);


}

