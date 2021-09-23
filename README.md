
# Prophesee_tool

My own tools for prophesee processing.  
(Add it as a submodule to your project.)


## Tool: bag2txt
Convert a rosbag to a txt file.

### Usage
First, create diretory for saving. Note that dirs must be created before running. For example,
```bash
mkdir ~/save_dir
mkdir ~/save_dir/image
rosrun prophesee_tool bag2txt --input_bag=<input/bag/name> --output_dir=<save_dir>
```

`input_bag`: bag to be converted

`output_dir`: images/events to be saved:
-    `events.txt`: all events in bag, format: tx x y polarity
-    `image/xxx.bmp`: all images indexed from 1
-    `images.txt`: image index and timestamp in `image/xxx.bmp`, format: idx ts 


## Tool: raw2csv
Convert a .raw file to a txt.  
Modified from prophesee-ai.

### Usage
```bash
rosrun prophesee_tool raw2csv -i <input/rawfile/name> -o <output/filename>
```
