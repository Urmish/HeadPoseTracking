#!/bin/bash
cd /home/urmish/Documents/OpenCV/Practice/configs/matcher
array=$(ls "/home/urmish/Documents/OpenCV/Practice/configs/DataFilters/")
knn=(1 5 10)
echo ${knn[@]}
echo $array
epsilon=(0 4 8)
for file in ${array}
do for val1 in ${knn[@]}
	do for val2 in ${epsilon[@]}
		do
			file2=$(echo "${val1}_${val2}_${file}")
			echo $file2
			full_file1=$(echo "/home/urmish/Documents/OpenCV/Practice/configs/DataFilters/${file}")
			cp $full_file1 $file2
			sed -i "s/knn: 1/knn: $val1/" $file2
			sed -i "s/epsilon: 0/epsilon: $val2/" $file2
		done	
	done
	
done

