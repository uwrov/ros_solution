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

echo_and_run cd "/home/uwrov/ros_solution/src/wb_sol"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/uwrov/ros_solution/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/uwrov/ros_solution/install/lib/python3/dist-packages:/home/uwrov/ros_solution/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/uwrov/ros_solution/build" \
    "/usr/bin/python3" \
    "/home/uwrov/ros_solution/src/wb_sol/setup.py" \
    egg_info --egg-base /home/uwrov/ros_solution/build/wb_sol \
    build --build-base "/home/uwrov/ros_solution/build/wb_sol" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/uwrov/ros_solution/install" --install-scripts="/home/uwrov/ros_solution/install/bin"
