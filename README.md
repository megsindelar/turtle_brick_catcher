# ME495 Embedded Systems Homework 2
Author: Megan Sindelar

1. Use `ros2 launch turtle_brick turtle_arena.launch.py` to run code
     -> Default runs a gui to control joint states
     -> Add `j_s_p:=____` to launch command with the blank as one of three choices:
        1. gui (default), a gui that controls joint states
        2. jsp, a joint state publisher node
        3. none, joints states are not controlled by a node
2. The `/place` service commands the brick to an initial location
3. The `/drop` service commands the brick to drop
4. Here is a video of two scenarios:
        1. Robot can get to the falling brick in time and catch it
        2. Robot can't get to the falling brick in time

[SindelarMegan_HW2_Demo.webm](https://user-images.githubusercontent.com/87098227/196987187-9f066a63-288e-4444-bbb0-687f02435c3c.webm)

I worked with: Liz Metzger, Marno Nel, Nick Morales, Ritika Ghosh, and Katie Hughes

References:

I referenced documents in 
https://github.com/ros2/launch_ros/blob/humble/launch_testing_ros/test/examples/talker_listener_launch_test.py
https://answers.ros.org/question/360264/ifconditionpythonexpression-am-i-doing-it-right/