# ray-tracing-project

This program attempts to simulate differing ways of perception. It presents three scenes, each of which highlights a form of perception.

Scene 1 consists of a bunny, elephant, table, and the cap of a circle (a very large one, to simulate the curvature of the ground). I used Perlin Noise to simulate the marble texture on the tabletop and to simulate grass and snow on the ground. I was able to scale, translate (through matrix transformations), and change the colors of each mesh object individually. Shadows are also implemented.

Scene 2 includes a sphere in addition to the mesh objects from Scene 1. This view is rendered by pointing the camera at the sphere, which reflects the objects. 

Scene 3 presents a monk (bottom left section of orb) in addition to the mesh objects from Scene 2. The monk has a sacred object near his head, and is facing the table, so we see the back of his head and shoulders. In addition to making the sphere reflective, I tried to implement Lambertian BRDF in order to make the orb glossier.

## Getting Started

You must install a C++ compiler and CMake (as a build system) before you can run the program. You may install CMake with a package manager; if on Debian/Ubuntu use ```$ sudo apt-get install cmake```, if on Mac use ```brew install cmake```, if on Windows use ```choco install -y cmake```.

### Compiling the Project

To view the result, please clone or download this repository and follow the commands:

1. Navigate to the proj directory and create a directory called build:
  ```
  $ cd proj; mkdir build
  ```
2. Use CMake to generate the Makefile/project files needed for compilation inside the build/ directory:
  ```
  $ cd build; cmake -DCMAKE_BUILD_TYPE=Release ..
  ```
3. Now we compile and run the compiled executable. The project consists of three scenes. 

  To render the first scene, enter:
  ```
  $ make; ./assignment4 ../data/scene1.json
  ```

  To render the second scene, enter:
  ```
  $ make; ./assignment4 ../data/scene2.json
  ```

  To render the third scene, enter:
  ```
  $ make; ./assignment4 ../data/scene3.json
  ```

## Acknowledgments

I worked with Professor Panozzo’s Assignment 4 solution as the foundation of my program.

I am using the Perlin Noise program under the terms of GPL v3. For more details see:
http://www.gnu.org/copyleft/gpl.html.

Mesh objects were found in https://github.com/kmammou/v-hacd-data/tree/master/data and https://github.com/gaschler/bounding-mesh/blob/master/examples/bunny/bunny.off.



