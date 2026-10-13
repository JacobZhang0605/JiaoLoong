
# README
---
**-Name:** Junming Zhang  
**-Student ID:** 525700910021  
**-Major:** ECE

## Project Description
-The folder includes a CAN project,in which basic skills are realized.It can drive a C620 whose id is 0x201. TIM6 and CAN1 are opened to receive and transmmit.
-The tile now enables c610 and m2026 to send a direct message for pid to control the loop and the speed of the motor  
-By excercises,the program can now turn 10 degree within 0.07s.

## Toolchain
Not included

## Attention
-Does not include a Changelog.md
-The parameter like ki,kp,feed-forward coefficient are RANDOMLY given  

## Bugs
-Sometimes it may jitter, with the spid's kp given as more than 0.49, it may be the critical point for the system.  
-The ppid's ki is 0, and after exercises, I found that it will strongly jitter if ki is not zero.

## Reference
-The given CAN&PID PPT