# ECE470ProjectBasketballShoot
Group members：Yiqun Niu, Zhenyu Zong, Xiang Li

This is update 4 for ECE 470 project.

### One previous group member dropped the course, so we only have three members in the group

## **1 Introduction**

Task：Demonstrate robot motion with integration of some decision making, planning, and perception. The robot will keep waiting until the player to pass item to it. We will make the robot throw the item in the future.

## **2 Methods**

- Make sure the documents ```vrep.py```, ```vrepConst.py``` and ```remoteApi.dll```(remoteApi.all for Window, for Mac users use ```remoteApi.dylib```) are in the path:

```
vrep_folder/programming/remoteApiBindings/
```

- The script code is in the file ```update4.py```. Run the code in a terminal with the Vrep simulator open.

### 2.1 Decision making

- The robot keeps waiting if nothing is passed to the Jacohand. If no item detected, send back response ```No items, wait!``` and wait. Otherwise, JacoHand will grasp the item and move it to the destination.

### 2.2 Decision making
end
- Then enable the ```JacoHand``` to grasp and release things. In ```update3.py``` file ```JacoHandGrasp``` function, send the command to ```JacoHand``` with codes:

```
vrep.simxSetStringSignal(clientID,'jacoHand','true',vrep.simx_opmode_oneshot)
vrep.simxSetStringSignal(clientID,'jacoHand','false',vrep.simx_opmode_oneshot)
```

- These instructions can open and close the hand.

## 3 Video 
Video can be found [here](https://youtu.be/Z8ZXQPDpTU8). 
