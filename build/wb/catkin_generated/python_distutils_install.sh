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

echo_and_run cd "/home/peter/ws/src/wb"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/peter/ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/peter/ws/install/lib/python3/dist-packages:/home/peter/ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/peter/ws/build" \
    "/usr/bin/python3" \
    "/home/peter/ws/src/wb/setup.py" \
    egg_info --egg-base /home/peter/ws/build/wb \
    build --build-base "/home/peter/ws/build/wb" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/peter/ws/install" --install-scripts="/home/peter/ws/install/bin"
