# Newton-M2-Ros-Driver

## How to use?
```
git clone <url>
```
### 1)Drop it into your workspace
```
catkin build
source devel/setup.bash
roslaunch starneto_mems starneto_mems.launch
```
### 2)Create a workspace and compile
```
mkdir catkin_ws
cd catkin_ws
mkdir src
catkin_init_workspace
# and Then goto 1)
```

## Msg
Input: $GPFPD or $GTIMU  
like:  
```
$GPFPD,2164,374485.800,261.108,0.862,-0.820,31.34534646,120.75819276,13.16,-0.006,-0.002,-0.008,2.047,30,31,4B*33
$GTIMU,2164,374485.500,0.0289,-0.0068,-0.0090,0.0137,0.0155,0.9935,53.0*63
```
Output: /GPFPD and /GTIMU  

## Config
```
node_rate: 100   # [Herz]
serial_port: /dev/ttyUSB0
GPFPD_output_topic: /GPFPD
GTIMU_output_topic: /GTIMU
```

