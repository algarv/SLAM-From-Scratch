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

echo_and_run cd "/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/install/lib/python3/dist-packages:/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/build" \
    "/usr/bin/python3" \
    "/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/install" --install-scripts="/home/algarverick/ME_495_SLAM/slam-project-algarv/homework_1/install/bin"
