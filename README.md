# powsybl-math-native
This project is the C++ implementation of [powsybl-math](https://github.com/powsybl/powsybl-core) `SparseMatrix` class, relying on SuiteSparse project.

## Requirements
To build `powsybl-math-native`, you need the following dependencies:
- [CMake](https://cmake.org/download)
- C++ compiler (gcc, clang or [Visual Studio](https://visualstudio.microsoft.com/fr/vs/features/cplusplus/))
- Java (8 or later)
- [Python](https://www.python.org/downloads) (2.7 or later), with [Six](https://pypi.org/project/six/) module
- [Lapack](http://www.netlib.org/lapack/) & [Blas](http://www.netlib.org/blas/) libraries

## Compilation

### Linux or MacOS
To build `powsybl-math-native`, run the following commands:
```
$> git clone https://github.com/powsybl/powsybl-math-native.git
$> cd powsybl-math-native
$> mkdir build
$> cd build
$> cmake ..
$> make 
$> cd ..
$> mvn install
````

### Windows
To build `powsybl-math-native`, run the following commands:
```
$> git clone https://github.com/powsybl/powsybl-math-native.git
$> cd powsybl-math-native
$> mkdir build
$> cd build
$> cmake .. -G "NMake Makefiles"
$> nmake 
$> cd ..
$> mvn install
````
