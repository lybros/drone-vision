Как собрать (Linux Ubuntu)

Установить библиотеки
Eigen
FreeImage
Qt
Ceres
Boost
OpenGL
Threads
JPEG

В папке с проектом:
mkdir release && cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_PREFIX_PATH=/home/kolina93/Qt/5.6/gcc_64/lib/cmake ..
make -j4

В DCMAKE_PREFIX_PATH установить свой путь до Qt
