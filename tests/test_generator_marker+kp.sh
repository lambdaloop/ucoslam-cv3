ORBSLAM2_PATH=/home/salinas/Libraries/ORB_SLAM2/trunk
ORBSLAM2_VOC=/home/salinas/Libraries/ORB_SLAM2/trunk/Vocabulary/ORBvoc.txt
UCOSLAM_PATH=/home/salinas/Libraries/ucoslam/build/
UCOSLAM_VOC=/home/salinas/Libraries/ucoslam/trunk/3rdparty/vocabularies/orb.fbow
DSO_PATH=/home/salinas/Libraries/dso/build/



SPM_PARAMS="-aruco-markerSize 0.165  -aruco-cornerRefinementM CORNER_LINES "
test[0]=SPM/video1/cam0
opt_args[0]="$SPM_PARAMS -aruco-dict ARUCO"
test[1]=SPM/video2/cam0
opt_args[1]="$SPM_PARAMS -aruco-dict ARUCO"
test[2]=SPM/video3/cam0
opt_args[2]="$SPM_PARAMS -aruco-dict ARUCO"
test[3]=SPM/video4/cam0
opt_args[3]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[4]=SPM/video5/cam0
opt_args[4]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[5]=SPM/video6/cam0
opt_args[5]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[6]=SPM/video7/cam0
opt_args[6]="$SPM_PARAMS -aruco-dict ARUCO"
test[7]=SPM/video8/cam0
opt_args[7]="$SPM_PARAMS -aruco-dict ARUCO"


test[8]=UcoCeiling/samsung7_markers/cam0
opt_args[8]="-aruco-markerSize 0.18  -aruco-cornerRefinementM CORNER_LINES  -maxFeatures 2000 -KFMinConfidence 0.6 -autoAdjustKpSensitivity -nokploopclosure -inplanemarkers"

nTests=9


function exec_test()
{
   dir=execdir-$METHODNAME-$DataSetName-$TESTNAME-$CAMERA-$( pwgen 20 1)
    echo "(ulimit -c unlimited;mkdir $dir -p; mkdir $RESULTSPATH -p;cd $dir; echo  \$(date) >log.txt; $1  >> log.txt ; echo \$(date) >>log.txt cd ..; ) "

}


function readDataSetName()
{
IFS='/' read -r -a array <<< "$1"
DataSetName="${array[0]}"
TESTNAME="${array[1]}"
CAMERA="${array[2]}"
}

#$1 pathToDataset 
#$2 additional arguments to the program
function execTest()
{
	readDataSetName $1
 	DATASETPATH=$TKESETPATH/$DataSetName
	RESULTSPATH=$RESULTS_DIR/$DataSetName
#	mkdir -p $RESULTSPATH
        exec_test "$SLAM_CMD $TKESETPATH/$1.mp4   $TKESETPATH/$1.yml $SLAM_VOC $RESULTSPATH/$TESTNAME@$CAMERA@0.log -noX $2"

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
  echo "Usage: method[orbslam2|dso|ucoslam] AbstPath2InputTkEDataset AbsPath2OutputResults [additonal options]"
  exit 1
fi



case "$1" in
    orbslam2)
		METHODNAME=$1
		SLAM_CMD=$ORBSLAM2_PATH/Examples/Monocular/mono_cvcam
		SLAM_VOC=$ORBSLAM2_VOC
    ;;
    ucoslam)
		METHODNAME=$1
		SLAM_CMD=$UCOSLAM_PATH/tests/test_sequence
		SLAM_VOC=$UCOSLAM_VOC
                METHOD_SPECIFIC_PARAMS=""

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
	RESULTS_DIR=$2
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
((i=0))
while((i<$nTests))
do
     execTest ${test[$i]} "$ADDITIONAL_ARGS ${opt_args[$i]} $METHOD_SPECIFIC_PARAMS"
     ((i++))

done


