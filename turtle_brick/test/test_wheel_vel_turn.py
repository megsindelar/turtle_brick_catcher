from turtle_brick.turtle_robot import wheel_vel_turn


def test_wheel_vel_turn():
    assert wheel_vel_turn(4.0, 4.0, 1.0, 1.0, 1.0) == (0.7853981633974483, 0.7853981633974483,
                                                       0.7071067811865476, 0.7071067811865475)
