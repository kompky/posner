Robot vision with OpenCV tutorial part one
==========================
[![GitPitch](https://gitpitch.com/assets/badge.svg)](https://gitpitch.com/vvv-school/tutorial_find-wally/master?grs=github&t=moon)

This first tutorial will guide you through integrating YARP with OpenCV and performing simple operations.

The task at hand is to play the world's renown Where's Wally? (or Where's Waldo? in the US).
The task consists of loading an illustration depicting dozens or more people doing a variety of amusing things at given locations. Users are challenged to find the character named Wally hidden in the group. Wally's distinctive red-and-white-striped shirt, bobble hat, and glasses make him slightly easier to recognise.

Tutorial
========

We aim to walk you through the steps contained in this code that will allow the users to complete the task at hand in a short amount of time using OpenCV.

How to run the module
========

##### Installing the module:

```
cd find-wally
mkdir build
cd build
cmake ..
make install
```

##### Running the module:

```
yarpmanager
# open the find-wally appication
# run and connect all modules
# then type:
yarp rpc /find-wally/rpc
load image.png
templateMatch wally.png 5
```
Correlation methods 
0=SQDIFF, 1=SQDIFF NORMED, 2=TM CCORR, 3=TM CCORR NORMED, 4=TM COEFF, 5=TM COEFF NORMED

For information on corresponence methods have a look at the following image:
![find-wally](/assets/methods.png)

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)
