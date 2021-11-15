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

echo_and_run cd "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/install/lib/python2.7/dist-packages:/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build" \
    "/usr/bin/python2" \
    "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/src/dynamixel_manipulation/setup.py" \
     \
    build --build-base "/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/build/dynamixel_manipulation" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/install" --install-scripts="/home/marcus/mintbox/multi-servo-dynamixel-library/mb-ws/install/bin"
