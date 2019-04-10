ORBSLAM2_PATH=/home/salinas/Libraries/ORB_SLAM2/trunk
ORBSLAM2_VOC=/home/salinas/Libraries/ORB_SLAM2/trunk/Vocabulary/ORBvoc.txt
UCOSLAM_PATH=/home/salinas/Libraries/ucoslam/build/
UCOSLAM_VOC=/home/salinas/Libraries/ucoslam/trunk/3rdparty/vocabularies/orb.fbow
DSO_PATH=/home/salinas/Libraries/dso/build/


test[0]=Euroc-MAV/mh_01
test[1]=Euroc-MAV/mh_02
test[2]=Euroc-MAV/mh_03
test[3]=Euroc-MAV/mh_04
test[4]=Euroc-MAV/mh_05
test[5]=Euroc-MAV/V1_01_easy
test[6]=Euroc-MAV/V1_02_medium
opt_args[6]="-KFMinConfidence 0.8 "
test[7]=Euroc-MAV/V1_03_difficult
opt_args[7]="-KFMinConfidence 0.8 "
test[8]=Euroc-MAV/V2_01_easy
test[9]=Euroc-MAV/V2_03_difficult
opt_args[9]="-KFMinConfidence 0.8 "


KITTI_ARGS="-KFMinConfidence 0.8 -KFCulling 0.8 -recovery"
test[10]=Kitti/00
opt_args[10]=$KITTI_ARGS
test[11]=Kitti/01
opt_args[11]="$KITTI_ARGS"
test[12]=Kitti/02
opt_args[12]=$KITTI_ARGS
test[13]=Kitti/03
opt_args[13]=$KITTI_ARGS
test[14]=Kitti/04
opt_args[14]=$KITTI_ARGS
test[15]=Kitti/05
opt_args[15]=$KITTI_ARGS
test[16]=Kitti/06
opt_args[16]=$KITTI_ARGS
test[17]=Kitti/07
opt_args[17]=$KITTI_ARGS
test[18]=Kitti/08
opt_args[18]=$KITTI_ARGS
test[19]=Kitti/09
opt_args[19]=$KITTI_ARGS

nTests=20

function exec_test()
{
                dir=execdir-$METHODNAME-$DataSetName-$TESTNAME-$( pwgen 20 1)
                echo "(mkdir $dir -p; mkdir $RESULTSPATH -p;cd $dir; echo  \$(date) >log.txt; $1  >> log.txt ; echo \$(date) >>log.txt cd ..; ) "

}

function readDataSetName()
{
IFS='/' read -r -a array <<< "$1"
DataSetName="${array[0]}"
TESTNAME="${array[1]}"
}

#$1 pathToDataset 
#$2 additional arguments to the program
function execTest()
{
	readDataSetName $1
 	DATASETPATH=$TKESETPATH/$DataSetName
	RESULTSPATH=$RESULTS_DIR/$DataSetName
#	mkdir -p $RESULTSPATH
        exec_test "$SLAM_CMD $TKESETPATH/$1/cam0.mp4 $TKESETPATH/$1/stereo.yml $SLAM_VOC $RESULTSPATH/$TESTNAME@0.log -noX -stereo $TKESETPATH/$1/cam1.mp4 $2"

}
function getADDITIONAL_ARGS(){
	ADDITIONAL_ARGS=""
	ArgArray=( "$@" )
	for((i=3;i<$#;i++))
	do 
		ADDITIONAL_ARGS="$ADDITIONAL_ARGS ${ArgArray[$i]}"
	done
}

if [ $# -lt 3 ]; then
  echo "Usage: method[orbslam2|dso|ucoslam] AbstPath2InputDataset AbsPath2OutputResults [additonal options]"
  exit 1
fi



case "$1" in
    orbslam2)
		METHODNAME=$1
                SLAM_CMD=$ORBSLAM2_PATH/Examples/Stereo
		SLAM_VOC=$ORBSLAM2_VOC
    ;;
    ucoslam)
		METHODNAME=$1
		SLAM_CMD=$UCOSLAM_PATH/tests/test_sequence
		SLAM_VOC=$UCOSLAM_VOC
                #METHOD_SPECIFIC_PARAMS="-nomarkers"
    ;;
    dso)
		METHODNAME=$1		
		SLAM_CMD=$DSO_PATH/bin/dso_dataset
		SLAM_VOC=
    ;;
    *)
		echo "Invalid first argument: $1"
		exit		
esac

case $2 in
	/*)
	TKESETPATH=$2
	;;
	*) 
	path=$(pwd)
	TKESETPATH=$path/$2 
esac

case $3 in
	/*)
        RESULTS_DIR=$3 #$2
	;;
	*) 
	path=$(pwd)
	RESULTS_DIR=$path/$3 

esac

RESULTS_DIR=$RESULTS_DIR/$METHODNAME
getADDITIONAL_ARGS $@ 

#set nomarkers for standard dataset KITTI, TUM and EUROC
for((i=0;i<$nTests;i++))
do
opt_args[$i]="${opt_args[$i]} "
done
SLAM_CMD_EUROC=$SLAM_CMD/stereo_euroc
SLAM_CMD_KITTI=$SLAM_CMD/stereo_kitti
((i=0))
while((i<$nTests))
do
     if [ "$METHODNAME" = "orbslam2" ];then
        if [ $i -lt 10 ] ; then
            SLAM_CMD=$SLAM_CMD_EUROC
        else
            SLAM_CMD=$SLAM_CMD_KITTI
        fi
     fi
     execTest ${test[$i]} "$ADDITIONAL_ARGS ${opt_args[$i]} $METHOD_SPECIFIC_PARAMS"
     ((i++))

done


