#!/bin/bash
function getADDITIONAL_ARGS(){
	ADDITIONAL_ARGS=""
	ArgArray=( "$@" )
	for((i=2;i<$#;i++))
	do 
		ADDITIONAL_ARGS="$ADDITIONAL_ARGS ${ArgArray[$i]}"
	done
}
 function getADDITIONAL_ARGS_STRING(){
	ADDITIONAL_ARGS_STRING=""
	ArgArray=( "$@" )
	for((i=2;i<$#;i++))
	do 
 		ADDITIONAL_ARGS_STRING="$ADDITIONAL_ARGS_STRING"_"${ArgArray[$i]}"
	done
}
if [ $# -lt 2 ]; then
  echo "Usage: [revision|current] NProc [additonal options]"
  exit 1
fi

cd /home/salinas/Libraries/ucoslam/trunk/

if [ "$1" = "current" ]; then
	svn up
	REVSTR=`svn info --show-item revision`
	REVSTR="$(echo -e "${REVSTR}" | sed -e 's/[[:space:]]*$//')"
else
	svn up -r $1
	REVSTR=$1
fi

cd /home/salinas/Libraries/ucoslam/build
make -j4
cd /home/salinas/execdir

getADDITIONAL_ARGS $@ 
getADDITIONAL_ARGS_STRING $@ 
OUTNAME="$REVSTR"_"$ADDITIONAL_ARGS_STRING"
mkdir exec_rev_$OUTNAME -p
cd exec_rev_$OUTNAME
rm * -rf

#echo "NN=$OUTNAME"
~/Libraries/ucoslam/trunk/tests/test_generator_monocular.sh ucoslam  ~/ucoslam-dataset  ~/results_$OUTNAME $ADDITIONAL_ARGS > ucoslam_script
~/Libraries/ucoslam/build/tests/task_manager ucoslam_script $2

