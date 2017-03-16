# LSD-SLAM: Large-Scale Direct Monocular SLAM

[![Codacy Badge](https://api.codacy.com/project/badge/Grade/c87a1633dad749e9aa47fd3c1809e7f9)](https://www.codacy.com/app/amarburg/lsd_slam?utm_source=github.com&utm_medium=referral&utm_content=amarburg/lsd_slam&utm_campaign=badger)
[![wercker status](https://app.wercker.com/status/4c30e195acc92af03c75e1f3451b6916/m/master "wercker status")](https://app.wercker.com/project/byKey/4c30e195acc92af03c75e1f3451b6916)

See my [Development Blog](https://faculty.washington.edu/amarburg/press/category/lsdslam/) for current status.

> __November/December 2016__ After early development, I'm trying to _reduce_ the number of external dependencies (introduced by myself or previous authors).   At a macro-scale I'm  experimenting with the [Conan](https://conan.io/) package manager.  However, building this repo __does not__ require Conan (though it can also be built with conan)

> One major step is removing the explicit dependency on Pangolin for the GUI.  Rather than introduce a package management nightmare (which I'm already dangerously close to anyway),  I've made the GUI a cmake-selectable option (`BUILD_GUI`).   It is __enabled__ by default in the standard `cmake` build, and __disabled__ by default in the Conan build.

> With `BUILD_GUI=False`, the core LSD-SLAM functionality is built into a library `liblsdslam` and the tool `LSD` is built.   This is a console-only app which spits out many log messages but isn't that exciting to watch.

> With `BUILD_GUI=True`, the current evolution of Tom Whelan's Pangolin GUI is built as `LSD_GUI`, with all of the dependencies that entails.

>  My full-fat, high-dependency work is continuing in [lsd_slam_conan](https://github.com/amarburg/lsd_slam_conan), which __requires__ Conan.

This fork started from [Thomas Whelan's fork](https://github.com/mp3guy/lsd_slam) which "relieves the user of the horrors of a ROS dependency and uses the much nicer lightweight [Pangolin](https://github.com/stevenlovegrove/Pangolin) framework instead."

Here is Jakob's original description:

> LSD-SLAM is a novel approach to real-time monocular SLAM. It is fully direct
> (i.e. does not use keypoints / features) and creates large-scale,
> semi-dense maps in real-time on a laptop. For more information see
> [http://vision.in.tum.de/lsdslam](http://vision.in.tum.de/lsdslam)
> where you can also find the corresponding publications and Youtube videos, as well as some
> example-input datasets, and the generated output as rosbag or .ply point cloud.

This repo contains my experiments with LSD-SLAM, for performance, functionality
and structure.   As of November 2016, it diverges significantly from either Jakob
or Thomas's branches in structure (I refactored as a way of learning the code),
but not significantly in terms of functionality (except for all the ways in which
I've broken it in the refactoring).   

**master**  is my working / stable-ish branch.   **aaron_dev** is my **really unstable** branch.   **Please note: BOTH BRANCHES ARE MOVING TARGETS.**  it's just that **aaron_dev** is, uh, moving faster.

# 1. Quickstart

My targeted environments are Ubuntu 16.04, the [Jetson TX1](http://www.nvidia.com/object/jetson-tx1-module.html) using [NVidia Jetpack 2.3](https://developer.nvidia.com/embedded/jetpack) , and OS X 10.12 with [Homebrew](http://brew.sh/).

The most authoritative documentation is stored in the Ruby Rakefile (don't be scared, it's
pretty readable).   This includes tasks for installing dependencies (in Travis and Docker images for example),
and for automating building and testing.

Assuming all of the "standard" (apt-gettable/Brew-able) deps have been installed, then a standard-ish cmake-ish:

    mkdir build
    cd build/
    cmake ..
    make deps
    make
    make unit_test

Or

    rake dependencies:{xenial,trusty,osx}:gui
    rake {debug,release}:test

Should work.

For the conan-based build:

    rake conan:dependencies:{xenial,trusty, osx}
    rake conan:debug:test


In addition to a number of "standard" (apt-gettable) dependencies,
LSD-SLAM uses these "non-standard" dependencies:
 * [g2o](https://github.com/RainerKuemmerle/g2o)
 * [g3log](https://github.com/KjellKod/g3log)
 * (Optionally) [Google Test](https://github.com/google/googletest) for unit testing
 * [Pangolin](https://github.com/stevenlovegrove/Pangolin) is the GUI is enabled.

LSD-SLAM will use CMake ExternalProjects to build each of these
dependencies automatically.  **This no longer happens automatically as part
of a `make` or `make all` ---** it was taking too long to re-check the dependencies
every time.   Instead, `make dep` should be run the first time.  This will
build just the dependencies.  CMake will (still) not resolve these dependencies
correctly when building in parallel ('make -j').

Set the appropriate CMake variable `BUILD_LOCAL_* = OFF` to disable building
local copies.  If you want to build G2O, Pangolin, etc. yourself, see
the `cmake/Build*` files for the CMake flags I used.

See also [doc/CommonProblems.md](doc/CommonProblems.md)

# 4. Running

Supports directories or sets of raw images. For example, you can download
any dataset from [here](http://vision.in.tum.de/lsdslam), and run:

    ./LSD -c datasets/LSD_machine/cameraCalibration.cfg -f datasets/LSD_machine/images/

I've started to document my performance testing in [doc/Performance.md](doc/Performance.md)

# 5. Related Papers

* **LSD-SLAM: Large-Scale Direct Monocular SLAM**, *J. Engel, T. Schöps, D. Cremers*, ECCV '14

* **Semi-Dense Visual Odometry for a Monocular Camera**, *J. Engel, J. Sturm, D. Cremers*, ICCV '13

# 6. License

LSD-SLAM is licensed under the GNU General Public License Version 3 (GPLv3), see http://www.gnu.org/licenses/gpl.html.

# 7. TODOS
