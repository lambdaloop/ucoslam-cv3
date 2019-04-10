#include "ucoslam.h"
#include "utils/system.h"
#include "basictypes/debug.h"
namespace ucoslam{
UcoSlam::UcoSlam(){
    impl=new System;
}
UcoSlam::~UcoSlam(){
    delete (System*)impl;
}
void UcoSlam::setParams(std::shared_ptr<Map> map, const  ucoslam::Params &params, const std::string &vocabulary){
    reinterpret_cast<System*>(impl)->setParams(map,params,vocabulary);
}
void UcoSlam::clear(){
    reinterpret_cast<System*>(impl)->clear();
}
Params  & UcoSlam::getParams() {
    return System::getParams();
}
cv::Mat UcoSlam::process(  cv::Mat &in_image,const ImageParams &ip,uint32_t frameseq_idx){
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx);
}
cv::Mat UcoSlam::processStereo( cv::Mat &in_image,const cv::Mat &R_image,const ImageParams &ip,uint32_t frameseq_idx){
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx,cv::Mat(),R_image);
}
cv::Mat UcoSlam::processRGBD( cv::Mat &in_image,const cv::Mat & depth,const ImageParams &ip,uint32_t frameseq_idx){
    return  reinterpret_cast<System*>(impl)->process(in_image,ip,frameseq_idx,depth);
}
void UcoSlam::resetTracker(){
    reinterpret_cast<System*>(impl)->resetTracker();
}
void UcoSlam::setMode(MODES mode){
    reinterpret_cast<System*>(impl)->setMode(mode);
}
//void UcoSlam::resetCurrentPose(){
//    reinterpret_cast<System*>(impl)->resetCurrentPose();
//}
uint32_t UcoSlam::getLastProcessedFrame()const{
    return  reinterpret_cast<System*>(impl)->getLastProcessedFrame();
}
void UcoSlam::saveToFile(std::string filepath){
    reinterpret_cast<System*>(impl)->saveToFile(filepath);
}
void UcoSlam::readFromFile(std::string filepath){
    reinterpret_cast<System*>(impl)->readFromFile(filepath);
}
void UcoSlam::globalOptimization(){
    reinterpret_cast<System*>(impl)->globalOptimization();
}
//waits for all threads to finish
void UcoSlam::waitForFinished(){
    reinterpret_cast<System*>(impl)->waitForFinished();
}
uint32_t UcoSlam::getCurrentKeyFrameIndex(){
    return reinterpret_cast<System*>(impl)->getCurrentKeyFrameIndex();
}
std::shared_ptr<Map> UcoSlam::getMap(){
    return reinterpret_cast<System*>(impl)->getMap();
}
void UcoSlam::setDebugLevel(int level){
    debug::Debug::setLevel(level);
}
void UcoSlam::showTimers(bool v){
    debug::Debug::showTimer(v);
}
void UcoSlam::updateParams(const Params &p){
}
std::string UcoSlam::getSignatureStr()const{
    return reinterpret_cast<System*>(impl)->getSignatureStr();
}
}
