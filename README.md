# Machine-Learning-and-Pattern-Recognition
This step by step guide is for Windows only but this project
could probably run on Linux aswell given that you have
the ***PCL*** library installed.

## Required programs/libraries
- [***Chocolatey***](https://chocolatey.org/install)
- [***CMake***](https://cmake.org/download/) (Download the Windows x64 installer)
- [***PCL V.1.13.0***](https://github.com/PointCloudLibrary/pcl/releases)(Download the all in one exe)

## How to use

### Prequesties

Download all of the programs/libraries listed above

When installing PCL, make sure to check the checkbox that add the root folder to your *environmental variables*.

Next open up the path:
```
C:\Program Files\PCL 1.13.0\3rdParty\OpenNI2
```

in a file explorer and run the .msi file.

Go to your environmental variables
First verify that under *System variables* you have a variable called PCL_ROOT.

Next double click on the *Path* variable and add the following:
```
C:\Program Files\CMake\bin
C:\Program Files\OpenNI
C:\Program Files\PCL 1.13.0\bin
C:\Program Files\PCL 1.13.0\3rdParty
C:\Program Files\PCL 1.13.0\3rdParty\VTK\bin
C:\Program Files\OpenNI2\Redist
```

Lastly download or clone
the repository.

### Compiling

To compile the code,
Open up your favorite terminal
create a ***build*** folder

```
mkdir build
```

then navigate to it:
```
cd build
```

Create the compilation files:
```
cmake ..
```

and compile:

```
cmake --build .
```

### Running the program

If everything went without errors
you should have a ***MLPR.exe*** file and two csv files in the folder 
```
./build/Debug/
```
Double click to run it and it should visualize a gesture. You might have to find it by holding in left-click and dragging your mouse to the left (for some reason the model isn't centered)






