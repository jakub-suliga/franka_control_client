import time
import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), "..", "src"))

from franka_control_client.device.robot.franka import (
    RemoteFranka,
    ControlMode,
)
from franka_control_client.core.exception import CommandError, DeviceNotReadyError


LEADER_ROBOT_IP = "127.0.0.1"
LEADER_ROBOT_PORT = 5555

FOLLOWER_ROBOT_IP = "127.0.0.1"
FOLLOWER_ROBOT_PORT = 5556


def run_leader_follower_example():
    client_leader = RemoteFranka(LEADER_ROBOT_IP, LEADER_ROBOT_PORT)
    client_follower = RemoteFranka(FOLLOWER_ROBOT_IP, FOLLOWER_ROBOT_PORT)

    try:
        client_follower.connect()
        client_leader.connect()
        leader_port = client_leader.get_sub_port()
        client_leader.start_control(ControlMode.HUMAN_MODE)
        client_follower.start_control(
            ControlMode.JOINT_VELOCITY,
            controller_ip=LEADER_ROBOT_IP,
            controller_port=leader_port,
        )

        while True:
            try:
                print(client_follower.get_state())
            except KeyboardInterrupt:
                break

    except Exception as e:
        print(e)
    finally:
        client_follower.disconnect()
        client_leader.disconnect()


if __name__ == "__main__":
    run_leader_follower_example()
