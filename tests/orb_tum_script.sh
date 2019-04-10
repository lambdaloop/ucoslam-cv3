DATASET_PATH=/home/salinas/tke_dataset/
DSO_CMD=/home/salinas/Libraries/dso/build/bin/dso_dataset
RESULTS_DIR=/home/salinas/results

ORB_PATH=/home/salinas/Libraries/ORB_SLAM2-master/Examples/Monocular
commExec=$ORB_PATH/mono_tum 
commResult=/home/salinas/Libraries/slamuco/build-trunk-Desktop-Release/tests/tum/comparelogs

for entry in "$indir"/*
do
	for i in {0..4}
	do
		echo  $entry
		# $outdir/orbslam2_$(basename $entry)_$i.log
		#echo $(basename $entry)_$i >> $outdir/results
		#$commResult  $entry/groundtruth.txt  $outdir/orbslam2_$(basename $entry)_$i.log >> $outdir/results
	done
done
