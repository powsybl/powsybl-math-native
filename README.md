# Linux build

Everything is automated on Linux using a Dockefile. To build Docker image, run the following command (proxy related arguments are  optionals).

```bash
docker build -t powsybl-math-native --build-arg proxy_host=<HOST> --build-arg proxy_port=<PORT> --build-arg proxy_username=<USER> --build-arg proxy_password=<PWD> .
```

Once docker image has  been built, we can retrieved native jar:

```bash
docker run --name powsybl-math-native-tmp powsybl-math-native /bin/true
docker cp powsybl-math-native-tmp:/powsybl-math-native/build/powsybl-math-linux_64-1.0.0.jar /tmp
docker rm powsybl-math-native-tmp
```

# Windows build

Following components have to be installed:

 - Git for Windows: https://gitforwindows.org/
 - CMake >= 3.1: https://cmake.org/download/
 - Python 2.7 for Windows: https://www.python.org/downloads/windows/
 - Visual studio c++ compiler community edition: https://visualstudio.microsoft.com/fr/vs/features/cplusplus/

# Jar installation

To install jar in local repository:

```bash
mvn install:install-file -Dfile=powsybl-math-linux_64-1.0.0.jar -DgroupId=com.powsybl -DartifactId=powsybl-math-linux_64 -Dversion=1.0.0 -Dpackaging=jar
```
