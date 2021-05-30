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

## Running the first experiment 
---
```
$ cd bachelor-thesis/argos3-test
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
The mass and the friction can be changed in the [test.argos](https://github.com/knudsen97/bachelor-thesis/blob/master/argos3-test/experiments/test.argos) file.

## Running the second experiment 
---
```
$ cd bachelor-thesis/epuck-simulation
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
## Running the third experiment 
---
```
$ cd bachelor-thesis/epuck-simulation
$ cmake -H./ -B./build 
$ cmake --build ./build --target all
$ argos3 -c experiments/test.argos 
```
