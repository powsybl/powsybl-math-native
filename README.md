# Linux build

Everything is automated on Linux using a Dockerfile. To build Docker image, run the following command (proxy related arguments are  optionals).

```bash
docker build -t powsybl-math-native --build-arg proxy_host=<HOST> --build-arg proxy_port=<PORT> --build-arg proxy_username=<USER> --build-arg proxy_password=<PWD> .
```

Once docker image has been built, we can retrieved native jar:

```bash
docker run --name powsybl-math-native-tmp powsybl-math-native /bin/true
docker cp powsybl-math-native-tmp:/powsybl-math-native/build/powsybl-math-linux_64-1.0.0.jar /tmp
docker rm powsybl-math-native-tmp
```

# Windows build

Following components have to be installed:

 - Git for Windows: https://gitforwindows.org/
 - CMake >= 3.1: https://cmake.org/download/. It must be added to the path.
 - Python 2.7 for Windows: https://www.python.org/downloads/windows/
   - Python27 must be added to the path
   - Six module must be installed. In a cmd.exe shell run the following command: 
```bash 
python -m pip install six
```
 - Visual studio c++ compiler community edition: https://visualstudio.microsoft.com/fr/vs/features/cplusplus/

# Jar installation

To install jars in local repository:

```bash
mvn install
```
