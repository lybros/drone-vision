# drone-vision

Course work on [FAMCS](http://www.fpmi.bsu.by/en/main.aspx),
[BSU](http://www.bsu.by/en/main.aspx) and [UIIP](http://uiip.bas-net.by/eng/),
[NASB](http://nasb.gov.by/eng/)

### requirements:

- [Qt](https://www.qt.io/download/) ~5.7
- [cmake](https://cmake.org/download/) ~3.5.2
- [OpenCV](https://www.gittip.com/OpenCV/) ~2.4.13
- [FreeImage](http://freeimage.sourceforge.net/) ~3.17.0
- [Eigen](eigen.tuxfamily.org/) ~3.2.10
- [Ceres-solver](http://ceres-solver.org/building.html) ~1.11.0
- [Glog](https://github.com/google/glog) ~0.3.4
- [Boost](http://www.boost.org/) ~1.60.0
- [Glew](http://glew.sourceforge.net/install.html) ~2.0.0
- [jpeg](http://libjpeg.sourceforge.net/) ~8d
- OpenGL
- Threads

### install:

#### for mac os:

```
bash mac_os_install.sh
```

install all requirements via [Homebrew](http://brew.sh/), you can switch on
needed version with `brew switch <formula> <version>`

### build:

set `<path_to_qt_cmake>` for example `$HOME/Qt/5.7/clang_64/lib/cmake/` set
`(CMAKE_PREFIX_PATH, <path_to_qt_cmake>)` in `CMakeLists.txt:9` and run:

```
mkdir release && cd release
cmake -D CMAKE_BUILD_TYPE=RELEASE -DCMAKE_PREFIX_PATH=<path_to_qt_cmake> ..
make -j4
```

or just run:

```
bash build.sh
```

### run:

```
bash run.sh
```

or

```
./release/3d_reconstruction
```

### development:

- import code style settings for JetBrains IDEs from `CodeStyle.xml`

- use `utils/find_obj.py` for bruteforce matching images, usage:
  ```
  python find_obj.py <image_to_find> # --feature=orb --in=in/ --out=out/ by default
  ```
