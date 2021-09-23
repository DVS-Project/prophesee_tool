
#!/bin/bash

# convert all .raw file to .csv file

input_dir=/home/larrydong/led/
output_dir=/home/larrydong/led/csv
raw_files=(`ls $input_dir | grep ".raw" | sort -d`)   # make an array

idx=0
while [ $idx -le 5 ]
do
    raw_file=${raw_files[$idx]}
    echo "idx: $idx, raw_file: $raw_files"

    # run convert file
    rosrun prophesee_tool raw2csv -i $input_dir/$raw_file -o tmp
    `mv tmp.csv $output_dir/${raw_file%.*}.csv`
    echo "Finished. "

    idx=$[$idx + 1]
done
echo "Move all bags to file: $bag_save_dir"
