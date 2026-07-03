# powsybl-math-native
This project is the C++ implementation of [powsybl-math](https://github.com/powsybl/powsybl-core) `SparseMatrix` class, relying on SuiteSparse project.

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

## Optional cuDSS (GPU) LU decomposition

An optional NVIDIA [cuDSS](https://docs.nvidia.com/cuda/cudss/) backend provides a
GPU LU decomposition (`CuDssLUDecomposition` / `CuDssMatrixFactory` in
powsybl-core, used through powsybl-open-loadflow). It is built as a **separate**
shared library `libmathcudss`, so the CPU-only `libmath` never depends on CUDA.

It is **off by default and opt-in**. To build and bundle it (Linux x86_64):

```
$> cmake .. -DWITH_CUDSS=ON -DCUDSS_ROOT=/path/to/cudss -DCUDAToolkit_ROOT=/path/to/cuda
$> make
$> cd ..
$> mvn install        # bundles libmathcudss into the jar, like libmath
```

`WITH_CUDSS` controls both building and bundling: when ON, `libmathcudss` is built
into `target/classes/natives/<arch>/` and packaged into the jar (so the GPU backend
is selectable with no manual staging); when OFF, it is never produced.

**On redistribution.** `libmathcudss` is our own small JNI wrapper (MPL) and is
*dynamically* linked to cuDSS, so a jar that contains it never contains NVIDIA's
`libcudss.so` — only a runtime reference to it. Our wrapper is therefore freely
distributable; what we must not redistribute is `libcudss.so` itself, which is
never bundled. (Today CI also has no CUDA at build time, so released jars contain
no cuDSS at all.)

**Runtime.** Loading `libmathcudss` needs `libcudss`, `libcudart` and `libcublas`
reachable by the dynamic linker, plus an NVIDIA GPU. When unavailable,
`CuDssMatrixFactory.isAvailable()` returns false and callers fall back to KLU.
There are two ways to make the runtime libraries reachable:

- set `LD_LIBRARY_PATH` to the cuDSS and CUDA `lib` directories, **or**
- build with `-DCUDSS_EMBED_RPATH=ON` to bake an absolute `RPATH` to those
  directories into `libmathcudss`, so the jar runs with **no `LD_LIBRARY_PATH`**
  (convenient for a local GPU build consumed by open-loadflow). This embeds a
  machine-local absolute path, so it is for local use, not a portable jar:

  ```
  $> cmake .. -DWITH_CUDSS=ON -DCUDSS_EMBED_RPATH=ON \
         -DCUDSS_ROOT=/path/to/cudss -DCUDAToolkit_ROOT=/path/to/cuda
  ```

A fully self-contained jar that also bundles `libcudss.so` (and its CUDA deps) so
it is portable to other machines is possible but heavy (hundreds of MB) and would
need a custom native loader; it is not implemented.
