# ray-tracing-project


1. Navigate to the proj directory and create a directory called build:
  $ cd proj; mkdir build
2. Use CMake to generate the Makefile/project files needed for compilation inside the build/ directory:
  $ cd build; cmake -DCMAKE_BUILD_TYPE=Release ..
3. Now we compile and run the compiled executable. The project consists of three scenes. 

  To render the first scene, enter:
  $ make; /assignment4 ../data/scene1.json

  To render the second scene, enter:
  $ make; /assignment4 ../data/scene2.json

  To render the third scene, enter:
  $ make; /assignment4 ../data/scene3.json



Important information:

I worked with Professor Panozzo’s Assignment 4 solution as the foundation of my program.

I am using the Perlin Noise program under the terms of GPL v3. For more details see:
http://www.gnu.org/copyleft/gpl.html.
Mesh objects were found in https://github.com/kmammou/v-hacd-data/tree/master/data and https://github.com/gaschler/bounding-mesh/blob/master/examples/bunny/bunny.off.
