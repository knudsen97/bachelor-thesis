# About bachelor-thesis
This is the source code for a bachelor project made by students at University of Southern Denmark.
# Running the project
To run this project you need to download: 
* [ARGoS](https://github.com/ilpincy/argos3)
* [OpenCV](https://github.com/opencv/opencv)

After ARGoS and OpenCV is installed, clone this repository. 
```
$ git clone https://github.com/knudsen97/bachelor-thesis
```

## Running the e-puck test
---
```
$ cd bachelor-thesis/argos3-test
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
The mass and the friction can be changed in the [argos3-test/experiments/test.argos](https://github.com/knudsen97/bachelor-thesis/blob/master/argos3-test/experiments/test.argos) file.

## Running simulation sorting closest objects
---
```
$ cd bachelor-thesis/epuck-simulation
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
The amount of robots and objects can be added or removed from the [epuck-simulation/experiments/test.argos](https://github.com/knudsen97/bachelor-thesis/blob/master/epuck-simulation/experiments/test.argos) file.

Remember to change the `clients` number to fit the amount of robots.

## Running simulation sorting objects in random
---
```
$ cd bachelor-thesis/no_sort
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
The amount of robots and objects can be added or removed from the [no_sort/experiments/test.argos](https://github.com/knudsen97/bachelor-thesis/blob/master/epuck-simulation/experiments/test.argos) file.

Remember to change the `clients` number to fit the amount of robots.
