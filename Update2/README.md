# ECE470ProjectBasketballShoot
Group members：Yiqun Niu, Zhenyu Zong, Xiang Li

This is update 3 for ECE 470 project.

### One previous group member dropped the course, so we only have three members in the group

## **1 Introduction**

Task：Demonstrate forward kinematics and progress towards getting our robot to achieve our task.

## **2 Methods**

- Make sure the documents ```vrep.py```, ```vrepConst.py``` and ```remoteApi.dll```(remoteApi.all for Window, for Mac users use remoteApi.dylib) are in the path:

```
vrep_folder/programming/remoteApiBindings/
```

### 2.1 Forward Kinematics

- The script code is in the file ```update3.py```. Run the code in a terminal with the Vrep simulator open, the arm will initialize its position and take commands from the user. The user should type in six angles in degrees which be added to the current angle to move the UR3 arm. You have at most ten chances to try to move the arm. The transform matrix from the home position to the new position will be printed on the screen, which demostrates forward kinematics.

### 2.2 JacoHand Gripper

- The hand is based on child script. In JacoHand's child script file, we need to do some changes for enabling the hand to open and close. Codes are referenced from this [website](http://www.forum.coppeliarobotics.com/viewtopic.php?f=9&t=1891#p8135).

```
sig=sim.getStringSignal('jacoHand')
if sig~=nil then
	simClearStringSignal('jacoHand')
	if sig=='true' then closing=true else closing=false end
end
```

- Then enable the ```JacoHand``` to grap and release things. In ```update3.py``` file ```JacoHandGrap``` function, send the command to ```JacoHand``` with codes:

```
vrep.simxSetStringSignal(clientID,'jacoHand','true',vrep.simx_opmode_oneshot)
vrep.simxSetStringSignal(clientID,'jacoHand','false',vrep.simx_opmode_oneshot)
```

- These instructions can open and close the hand.

## 3 Video 
Video can be found [here](https://www.youtube.com/watch?v=oTiXtuupihU&feature=youtu.be). 
