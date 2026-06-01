# powsybl-math-native
This project provides the native (C++/JNI) backend for [powsybl-math](https://github.com/powsybl/powsybl-core), relying on the [SuiteSparse](https://github.com/DrTimothyAldenDavis/SuiteSparse) and [SUNDIALS](https://github.com/LLNL/sundials) projects.

It exposes:
- the `SparseMatrix` LU decomposition (KLU / CXSparse);
- the `Kinsol` non-linear solver (SUNDIALS KINSOL);
- a `GaussNewtonCholesky` weighted least-squares solver for the Gauss-Newton / Levenberg-Marquardt normal equations `(HᵀWH)Δx = HᵀWr`, backed by CHOLMOD.

CHOLMOD is built with its simplicial (BLAS-free) factorization so the library stays portable across Linux, macOS and Windows without a BLAS/LAPACK dependency.

## Requirements
To build `powsybl-math-native`, you need the following dependencies:
- [CMake](https://cmake.org/download)
- C++ compiler (gcc, clang or [Visual Studio](https://visualstudio.microsoft.com/fr/vs/features/cplusplus/))
- Java (11 or later)

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
