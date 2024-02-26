#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/vitaly/rtabmap_helper_ws/src/ros_numpy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/vitaly/rtabmap_helper_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/vitaly/rtabmap_helper_ws/install/lib/python3/dist-packages:/home/vitaly/rtabmap_helper_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/vitaly/rtabmap_helper_ws/build" \
    "/home/vitaly/miniconda3/bin/python3" \
    "/home/vitaly/rtabmap_helper_ws/src/ros_numpy/setup.py" \
     \
    build --build-base "/home/vitaly/rtabmap_helper_ws/build/ros_numpy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/vitaly/rtabmap_helper_ws/install" --install-scripts="/home/vitaly/rtabmap_helper_ws/install/bin"
