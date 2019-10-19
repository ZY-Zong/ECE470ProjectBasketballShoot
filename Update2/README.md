# ECE470ProjectBasketballShoot
Group members：Yiqun Niu, Zhenyu Zong, Xiang Li
This is update 2 for ECE 470 project.

## One previous group member dropped the course, so we only have three members in the group

## **1 Introduction**

Task：Demonstrate forward kinematics and progress towards getting our robot to achieve our task.

## **2 Methods**

- Download the V-rep and run it with

```
    $ ./vrep .sh
```
- Choose ```UR3.ttm``` and ```Jaco hand.ttm``` models from Model browser, connect them with Assemble / Disassemble icon. The scene is saved as ```Update2Scene.tttt``` file.

> - Delete default child script from the robot because we will run the script with outside python code.

- Make sure the documents ```vrep.py```, ```vrepConst.py``` and ```remoteApi.dll```(for Mac users, use .dylib) are in the path:

```
V-REP_PRO_EDU_V3_6_2_Mac/programming/remoteApiBindings/
```

- The script code is in the file ```update3.py```. Run the code in a terminal with the Vrep simulator open, the arm will initialize its position and take commands from the user. The user should type in six angles in degrees which be added to the current angle to move the UR3 arm. You have at most ten chances to try to move the arm. The transform matrix from the home position to the new position will be printed on the screen, whichi demostrates forward kinematics.

## 3 Video 
Video can be found [here](https://www.youtube.com/watch?v=oTiXtuupihU&feature=youtu.be). 
