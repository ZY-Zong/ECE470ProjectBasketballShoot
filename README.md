# ECE470ProjectBasketballShoot
Group mumbers：Yiqun Niu, Jieting Chen, Zhenyu Zong, Xiang Li
This is update 2 for ECE 470 project.

## **1 Introduction**

Task：Use V-rep simulator to do some basic robot movement and sensor measurements.

## **2 Methods**

- Download the V-rep and run it with

```
    $ ./vrep .sh
```
- Choose ```UR3.ttm``` and ```Jaco hand.ttm``` models from Model browser, connect them with Assemble / Disassemble icon. The scene is saved as ```Update2Scene.tttt``` file.

> - Delete default child script from the robot because we will run the script with outside python code.

- Make sure the documents ```vrep.py```, ```vrepConst``` and ```remoteApi.dylib``` are in the path:

```
V-REP_PRO_EDU_V3_6_2_Mac/programming/remoteApiBindings/python/python
```

- The script code is in the file ```example.py```. This file is based on the codes of the example given by ece470_vrep_Linux.

## **3 Script Codes**

- Make a martix of the position, which is the angle of every joint. 
```

```
- Then use SetJointPosition function to move the UR3 in simulator. The SetJointPosition function is 
formed by simxSetJointTargetPosition function, which can move only one joint in a particular angle
SetJointPosition function call simxSetJointTargetPosition function 6 time for the 6 joints. The three calls
set the six joints for three times.
