ORBSLAM2_PATH=/home/salinas/Libraries/ORB_SLAM2/
ORBSLAM2_VOC=/home/salinas/Libraries/ORB_SLAM2/Vocabulary/ORBvoc.txt
UCOSLAM_PATH=/home/salinas/Libraries/ucoslam/build/
UCOSLAM_VOC=/home/salinas/Libraries/ucoslam/trunk/3rdparty/vocabularies/orb.fbow
DSO_PATH=/home/salinas/Libraries/dso/build/
LDSO_PATH=/home/salinas/Libraries/LDSO.git/trunk/
LDSO_VOC=/home/salinas/Libraries/LDSO.git/trunk/vocab/orbvoc.dbow3

test[0]=Euroc-MAV/mh_01/cam0
test[1]=Euroc-MAV/mh_01/cam1
test[2]=Euroc-MAV/mh_02/cam0
test[3]=Euroc-MAV/mh_02/cam1
test[4]=Euroc-MAV/mh_03/cam0
test[5]=Euroc-MAV/mh_03/cam1
test[6]=Euroc-MAV/mh_04/cam0
test[7]=Euroc-MAV/mh_04/cam1
test[8]=Euroc-MAV/mh_05/cam0
test[9]=Euroc-MAV/mh_05/cam1
test[10]=Euroc-MAV/V1_01_easy/cam0
test[11]=Euroc-MAV/V1_01_easy/cam1
test[12]=Euroc-MAV/V1_02_medium/cam0
opt_args[12]="-KFMinConfidence 0.8 -KFCulling 0.9"
test[13]=Euroc-MAV/V1_02_medium/cam1
opt_args[13]="-KFMinConfidence 0.8 -KFCulling 0.9"
test[14]=Euroc-MAV/V1_03_difficult/cam0
opt_args[14]="-KFMinConfidence 0.8 -KFCulling 0.9"
test[15]=Euroc-MAV/V1_03_difficult/cam1
opt_args[15]="-KFMinConfidence 0.8 -KFCulling 0.9"
test[16]=Euroc-MAV/V2_01_easy/cam0
test[17]=Euroc-MAV/V2_01_easy/cam1
test[18]=Euroc-MAV/V2_03_difficult/cam0
opt_args[18]="-KFMinConfidence 0.8  -KFCulling 0.9"
test[19]=Euroc-MAV/V2_03_difficult/cam1
opt_args[19]="-KFMinConfidence 0.8  -KFCulling 0.9"




test[20]=TUM/fr1_desk/cam0
test[21]=TUM/fr1_desk2/cam0
test[22]=TUM/fr1_floor/cam0
test[23]=TUM/fr1_xyz/cam0
test[24]=TUM/fr2_360_kidnap/cam0
test[25]=TUM/fr2_desk/cam0
test[26]=TUM/fr2_desk_person/cam0
test[27]=TUM/fr2_xyz/cam0
test[28]=TUM/fr3_long_office/cam0
test[29]=TUM/fr3_nstr_tex_near/cam0

SPM_PARAMS="-aruco-markerSize 0.165  -aruco-cornerRefinementM CORNER_LINES -nomarkers"
test[30]=SPM/video1/cam0
opt_args[30]="$SPM_PARAMS -aruco-dict ARUCO"
test[31]=SPM/video2/cam0
opt_args[31]="$SPM_PARAMS -aruco-dict ARUCO"
test[32]=SPM/video3/cam0
opt_args[32]="$SPM_PARAMS -aruco-dict ARUCO"
test[33]=SPM/video4/cam0
opt_args[33]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[34]=SPM/video5/cam0
opt_args[34]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[35]=SPM/video6/cam0
opt_args[35]="$SPM_PARAMS -aruco-dict ARUCO_MIP_36h12"
test[36]=SPM/video7/cam0
opt_args[36]="$SPM_PARAMS -aruco-dict ARUCO"
test[37]=SPM/video8/cam0
opt_args[37]="$SPM_PARAMS -aruco-dict ARUCO"




KITTI_ARGS="-KFMinConfidence 0.8 -KFCulling 0.8 -recovery"
test[38]=Kitti/09/cam0
opt_args[38]=$KITTI_ARGS
test[39]=Kitti/09/cam1
opt_args[39]=$KITTI_ARGS
test[40]=Kitti/01/cam0
opt_args[40]="$KITTI_ARGS"
test[41]=Kitti/01/cam1
opt_args[41]="$KITTI_ARGS"
test[42]=Kitti/08/cam0
opt_args[42]=$KITTI_ARGS
test[43]=Kitti/08/cam1
opt_args[43]=$KITTI_ARGS
test[44]=Kitti/03/cam0
opt_args[44]=$KITTI_ARGS
test[45]=Kitti/03/cam1
opt_args[45]=$KITTI_ARGS
test[46]=Kitti/04/cam0
opt_args[46]=$KITTI_ARGS
test[47]=Kitti/04/cam1
opt_args[47]=$KITTI_ARGS
test[48]=Kitti/05/cam0
opt_args[48]=$KITTI_ARGS
test[49]=Kitti/05/cam1
opt_args[49]=$KITTI_ARGS
test[50]=Kitti/06/cam0
opt_args[50]=$KITTI_ARGS
test[51]=Kitti/06/cam1
opt_args[51]=$KITTI_ARGS
test[52]=Kitti/07/cam0
opt_args[52]=$KITTI_ARGS
test[53]=Kitti/07/cam1
opt_args[53]=$KITTI_ARGS
test[54]=Kitti/02/cam0
opt_args[54]=$KITTI_ARGS
test[55]=Kitti/02/cam1
opt_args[55]=$KITTI_ARGS
test[56]=Kitti/00/cam0
opt_args[56]=$KITTI_ARGS
test[57]=Kitti/00/cam1
opt_args[57]=$KITTI_ARGS
nTests=58


function exec_test()
{
                dir=execdir-$METHODNAME-$DataSetName-$TESTNAME-$CAMERA-$( pwgen 20 1)
                echo "(ulimit -c unlimited; mkdir $dir -p; mkdir $RESULTSPATH -p;cd $dir; echo  \$(date) >log.txt; $1  >> log.txt ; echo \$(date) >>log.txt cd ..; ) "

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
        exec_test "$SLAM_CMD $TKESETPATH/$1.mp4   $TKESETPATH/$1.yml $SLAM_VOC $RESULTSPATH/$TESTNAME@$CAMERA@0.log -noX -timefile $RESULTSPATH/$TESTNAME@$CAMERA@0.logtime $2"

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
  echo "Usage: method[orbslam2|dso|ldso|ucoslam] AbstPath2InputTkEDataset AbsPath2OutputResults [additonal options]"
  exit 1
fi



case "$1" in
    orbslam2)
		METHODNAME=$1
		SLAM_CMD=$ORBSLAM2_PATH/Examples/Monocular/mono_cvcam
		SLAM_VOC=$ORBSLAM2_VOC
                METHOD_SPECIFIC_PARAMS="-sequential"
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
    ldso)
                METHODNAME=$1
                SLAM_CMD=$LDSO_PATH/bin/run_dso_video
                SLAM_VOC=$LDSO_VOC
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
        RESULTS_DIR=$3
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


