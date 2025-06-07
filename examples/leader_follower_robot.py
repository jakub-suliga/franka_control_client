import time
import sys
import os

# Adjust the Python path to include the src directory
# This allows the script to find the franka_control_client package
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from franka_control_client.device.robot.franka import RemoteFranka, FrankaArmState, ControlMode
# MsgID might be useful if you want to interpret raw errors, but not directly used here.
from franka_control_client.core.exception import CommandError, DeviceNotReadyError

# --- Configuration ---
# IMPORTANT: These are placeholder IP addresses and ports.
# You MUST replace them with the actual IP addresses and ports of your robot control servers.
# If you are running a simulation (e.g., two instances of a test server),
# ensure they are configured to listen on these distinct IP/port combinations.
LEADER_ROBOT_IP = "127.0.0.1"  # Replace with actual leader robot control server IP
LEADER_ROBOT_PORT = 5555     # Replace with actual leader robot control server port

FOLLOWER_ROBOT_IP = "127.0.0.1" # Replace with actual follower robot control server IP
FOLLOWER_ROBOT_PORT = 5556   # Replace with actual follower robot control server port
# Note: If using a single machine to simulate two robots with a server program,
# that server program must be runnable twice on different ports, or it must be
# designed to simulate multiple robot instances reachable via different ports.
# For this example, we assume two separate server instances or two physically separate robots.

# --- Main Example Function ---
def run_leader_follower_example():
    print("Initializing leader and follower robot client instances...")
    # These objects are clients that will attempt to connect to robot control servers.
    leader_robot = RemoteFranka(LEADER_ROBOT_IP, LEADER_ROBOT_PORT)
    follower_robot = RemoteFranka(FOLLOWER_ROBOT_IP, FOLLOWER_ROBOT_PORT)

    try:
        print(f"Attempting to connect to leader robot at {LEADER_ROBOT_IP}:{LEADER_ROBOT_PORT}...")
        leader_robot.connect()
        print("Leader robot client connected to its server.")

        print(f"Attempting to connect to follower robot at {FOLLOWER_ROBOT_IP}:{FOLLOWER_ROBOT_PORT}...")
        follower_robot.connect()
        print("Follower robot client connected to its server.")

        # 1. Sanity Check: Ensure robots are in a controllable or queryable state.
        # The exact meaning of "status" or "mode" is dependent on the server implementation.
        # For a real Franka robot, this might involve checking if the robot is in "Guiding mode"
        # or if external control is active. Here, `get_status()` maps to `query_state()`
        # which returns the `ControlMode`.
        print("\nQuerying initial robot modes (statuses)...")
        leader_mode = leader_robot.get_status()
        follower_mode = follower_robot.get_status()
        print(f"Leader robot initial mode: {leader_mode.name if leader_mode else 'Unknown'}")
        print(f"Follower robot initial mode: {follower_mode.name if follower_mode else 'Unknown'}")

        # Conceptual Step: Prepare robots for leader-follower.
        # In a real scenario, you might need to switch both robots to a specific control mode
        # that allows for external joint position commands (e.g., ControlMode.JOINT_POSITION).
        # This would typically involve:
        # leader_robot.start_control(ControlMode.JOINT_POSITION, controller_ip="<leader_controller_ip>", ...)
        # follower_robot.start_control(ControlMode.JOINT_POSITION, controller_ip="<follower_controller_ip>", ...)
        # The `controller_ip` and `controller_port` would be for the ZMQ SUB socket where the
        # robot server expects this client to listen for control commands if it were sending them.
        # Since this example *reads* state and *conceptually* sends commands, full mode switching
        # isn't strictly necessary if the server allows state reading in its default mode.
        print("\n(Conceptual) Ensuring robots are in a mode suitable for external control if commands were to be sent.")

        print("\n--- Leader-Follower Emulation Loop Started ---")
        print("This loop simulates a leader robot's state being read, and a follower robot")
        print("conceptually attempting to mimic that state. No actual motion commands are")
        print("sent by this script to the `follower_robot` for simplicity and safety,")
        print("as it would require a server capable of processing such commands and a suitable environment.")
        print("Focus is on demonstrating client-side API usage for state reading.")

        for i in range(5): # Run for a few cycles
            print(f"\nCycle {i+1}:")

            # --- Leader's "Action" (Simulated by Reading State) ---
            # In a real application, the leader robot might be physically moved by an operator
            # (e.g., in a hand-guiding mode), or it might be executing a predefined trajectory.
            # For this example, we just fetch its current joint state.
            print("Leader: Reading its current joint state...")
            try:
                leader_state: FrankaArmState = leader_robot.get_state()
                # The `q` attribute of FrankaArmState typically holds the joint positions.
                print(f"Leader current joint positions (q): {[f'{x:.3f}' for x in leader_state.q]}")

                # --- Follower's "Reaction" (Conceptual) ---
                # The follower robot reads the leader's state and would, in a real system,
                # use this information (e.g., `leader_state.q`) as its target.
                print("Follower: Using leader's state as its target.")

                # This is where the follower would receive the leader's state as a command.
                target_q_for_follower = leader_state.q
                print(f"Follower's target joint positions (from leader): {[f'{x:.3f}' for x in target_q_for_follower]}")

                # CONCEPTUAL: If the follower robot were in an active control mode that accepts
                # joint position commands (e.g., JOINT_POSITION), you would send these
                # `target_q_for_follower` values to its controller.
                # Example (method does not exist in current RemoteFranka):
                # `follower_robot.send_target_joint_positions(target_q_for_follower)`
                #
                # For this example, we'll just fetch the follower's current state to show it's alive.
                # In a real system, its state would (ideally) start converging towards `target_q_for_follower`.
                follower_state: FrankaArmState = follower_robot.get_state()
                print(f"Follower current joint positions (q): {[f'{x:.3f}' for x in follower_state.q]}")
                print("Follower: (Conceptual) If it had a `send_target_joint_positions` method, it would have used it now.")

            except DeviceNotReadyError as e:
                print(f"DeviceNotReadyError during leader-follower cycle: {e}. This means a robot was not connected.")
                print("Aborting example.")
                break
            except CommandError as e:
                print(f"CommandError during leader-follower cycle: {e}")
                # Example: Server might return an error if robot is in a non-operational state.
                if "Server returned error code" in str(e) or "Frame too short" in str(e) or "payload size mismatch" in str(e):
                    print("Critical server communication error. Ensure the robot server is running correctly,")
                    print(f"is properly configured for IP/Port, and the robot is operational.")
                    break # Exit loop on critical errors
                # For other command errors, we might try to continue or implement more specific recovery.

            time.sleep(2) # Pause to simulate time between cycles.

        print("\n--- Leader-Follower Emulation Loop Finished ---")

    except ConnectionRefusedError:
        # This error occurs if the server is not running or not reachable at the specified IP/port.
        print(f"ERROR: Connection refused. Please ensure robot control server(s) are running and accessible at:")
        print(f"  Leader:   tcp://{LEADER_ROBOT_IP}:{LEADER_ROBOT_PORT}")
        print(f"  Follower: tcp://{FOLLOWER_ROBOT_IP}:{FOLLOWER_ROBOT_PORT}")
        print("If using a simulator or test server, ensure it's started and configured for these addresses.")
    except DeviceNotReadyError as e:
        # This can happen if connect() was successful but something made the robot not ready for commands
        # (e.g. if connect() was modified to not raise immediately on all failures)
        # or if a command is issued before connect()
        print(f"ERROR: Device not ready. {e}. Ensure connect() was called and succeeded for both robots.")
    except CommandError as e:
        # This catches other command errors that might occur during setup (e.g., during initial get_status)
        print(f"ERROR: A general command error occurred: {e}")
        print("This could be due to server-side issues or incorrect client-server communication.")
    except Exception as e:
        # Catch any other unexpected errors.
        print(f"ERROR: An unexpected error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n--- Cleanup Phase ---")
        print("Attempting to disconnect robot clients (if they were connected)...")
        # Check if the robot client object exists and if its socket was initialized before trying to disconnect.
        # The `_sock_req` attribute is set to a ZMQ socket object upon successful connection attempt in `connect()`,
        # and set to None in `__init__` and after `disconnect()`.
        if 'leader_robot' in locals() and leader_robot._sock_req is not None:
            try:
                leader_robot.disconnect()
                print("Leader robot client disconnected.")
            except Exception as e:
                print(f"Error disconnecting leader robot: {e}")
        else:
            print("Leader robot client was not connected or already disconnected.")

        if 'follower_robot' in locals() and follower_robot._sock_req is not None:
            try:
                follower_robot.disconnect()
                print("Follower robot client disconnected.")
            except Exception as e:
                print(f"Error disconnecting follower robot: {e}")
        else:
            print("Follower robot client was not connected or already disconnected.")
        print("Cleanup finished.")

if __name__ == "__main__":
    # --- Instructions for Running This Example ---
    # This script is a CLIENT-SIDE example. It needs SERVER(S) to connect to.
    #
    # 1. Server Setup:
    #    - You need one or two robot control server instances that the `RemoteFranka` class can communicate with.
    #    - These servers must listen on the IP addresses and ports defined by
    #      `LEADER_ROBOT_IP`, `LEADER_ROBOT_PORT`, `FOLLOWER_ROBOT_IP`, and `FOLLOWER_ROBOT_PORT`.
    #    - The server(s) should be able to handle:
    #        - Connection requests.
    #        - `MsgID.GET_STATE_REQ`: Responding with robot state data (like `FrankaArmState`).
    #        - `MsgID.QUERY_STATE_REQ`: Responding with the robot's current `ControlMode`.
    #    - If you have a `test_robot_server.py` or similar, you might need to:
    #        a) Run it twice, configured for the two different ports.
    #        b) Modify it to simulate two distinct robot instances if it's a single server.
    #
    # 2. Configuration:
    #    - Update `LEADER_ROBOT_IP`, `LEADER_ROBOT_PORT`, `FOLLOWER_ROBOT_IP`, `FOLLOWER_ROBOT_PORT`
    #      in this script to match your server setup.
    #
    # 3. Running the Script:
    #    - Execute this script from the root directory of the project (e.g., `python examples/leader_follower_robot.py`).
    #    - The `sys.path.append` line should allow it to find the `franka_control_client` library.
    #
    # Expected Behavior:
    #    - The script will attempt to connect to the two configured robot servers.
    #    - If successful, it will enter a loop, periodically fetching the leader's state
    #      and conceptually using that as a target for the follower.
    #    - It prints status messages throughout this process.
    #    - If servers are not available or an error occurs, it will print error messages.
    #    - This script DOES NOT send actual motion commands to the follower robot.
    #      It only simulates the logic by printing what it *would* do.
    #
    print("--- Leader-Follower Robot Example ---")
    print("This script demonstrates a conceptual leader-follower scenario using the RemoteFranka client.")
    print(f"Ensure robot control server(s) are running and configured for:")
    print(f"  Leader:   tcp://{LEADER_ROBOT_IP}:{LEADER_ROBOT_PORT}")
    print(f"  Follower: tcp://{FOLLOWER_ROBOT_IP}:{FOLLOWER_ROBOT_PORT}")
    print("Modify the IP addresses and ports in this script if needed.")
    print("This example focuses on state reading and does not send actual motion commands to the follower.")
    print("-" * 30)

    run_leader_follower_example()
# In a real scenario, these would be the actual IPs of your robot control servers.
LEADER_ROBOT_IP = "127.0.0.1"  # Replace with actual leader robot IP
LEADER_ROBOT_PORT = 5555     # Replace with actual leader robot port

FOLLOWER_ROBOT_IP = "127.0.0.1" # Replace with actual follower robot IP
FOLLOWER_ROBOT_PORT = 5556   # Replace with actual follower robot port
                               # Note: If using the same server instance, ports must be different
                               # and the server must support multiple robots.
                               # For this example, we assume two separate server instances/robots.

def run_leader_follower_example():
    print("Initializing leader and follower robots...")
    leader_robot = RemoteFranka(LEADER_ROBOT_IP, LEADER_ROBOT_PORT)
    follower_robot = RemoteFranka(FOLLOWER_ROBOT_IP, FOLLOWER_ROBOT_PORT)

    try:
        print("Connecting to leader robot...")
        leader_robot.connect()
        print("Leader robot connected.")

        print("Connecting to follower robot...")
        follower_robot.connect()
        print("Follower robot connected.")

        # 1. Ensure robots are in a controllable state (e.g., Human Mode or ready for control)
        # This step is highly dependent on the robot server's capabilities.
        # For a real Franka, you might need to switch to a specific control mode.
        # We'll query the mode for now.
        leader_mode = leader_robot.get_status()
        follower_mode = follower_robot.get_status()
        print(f"Leader robot initial mode: {leader_mode.name}")
        print(f"Follower robot initial mode: {follower_mode.name}")

        # Example: If robots need to be in joint position control mode
        # This is a conceptual step. Actual control mode switching would be:
        # leader_robot.start_control(ControlMode.JOINT_POSITION, controller_ip="...", controller_port=...)
        # follower_robot.start_control(ControlMode.JOINT_POSITION, controller_ip="...", controller_port=...)
        # print("Switched robots to JOINT_POSITION (conceptual).")

        print("\n--- Leader-Follower Emulation Started ---")
        print("Leader will 'virtually' move, Follower will attempt to 'mimic'.")
        print("This example primarily demonstrates state reading and conceptual following.")
        print("Actual robot movement commands are not sent to avoid needing a live, complex server setup.")

        for i in range(5):
            print(f"\nCycle {i+1}:")

            # --- Leader's "Action" ---
            # In a real scenario, the leader might be controlled by an operator or a script.
            # Here, we'll just fetch its current state.
            # If we could send commands, we'd send a move command to the leader.
            print("Leader: Getting its current state...")
            try:
                leader_state: FrankaArmState = leader_robot.get_state()
                print(f"Leader q: {[f'{x:.3f}' for x in leader_state.q]}")

                # --- Follower's "Reaction" ---
                # The follower reads the leader's state and tries to mimic it.
                # In a real application, the follower would receive `leader_state.q`
                # as its target joint positions.
                print("Follower: Reading leader's state and 'planning' to mimic.")

                # Conceptual: Follower receives leader_state.q as target
                target_q_for_follower = leader_state.q
                print(f"Follower's target q (from leader): {[f'{x:.3f}' for x in target_q_for_follower]}")

                # If follower was in a control mode, it would now try to move to `target_q_for_follower`.
                # e.g., follower_robot.send_joint_positions(target_q_for_follower) - this method doesn't exist yet.
                # For now, we just get follower's current state.
                follower_state: FrankaArmState = follower_robot.get_state()
                print(f"Follower current q: {[f'{x:.3f}' for x in follower_state.q]}")
                print("Follower: (Conceptual) Sent target joint positions to its controller.")

            except CommandError as e:
                print(f"Error during leader-follower cycle: {e}")
                # Decide if we should break or continue
                if "Server returned error code" in str(e) or "Frame too short" in str(e):
                    print("Critical server error. Check if the robot server is running and configured for these IPs/ports.")
                    break

            time.sleep(2) # Simulate time passing

        print("\n--- Leader-Follower Emulation Finished ---")

    except ConnectionRefusedError:
        print(f"Connection refused. Ensure robot server(s) are running at "
              f"{LEADER_ROBOT_IP}:{LEADER_ROBOT_PORT} and/or "
              f"{FOLLOWER_ROBOT_IP}:{FOLLOWER_ROBOT_PORT}.")
    except CommandError as e:
        print(f"A command error occurred: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        print("\nDisconnecting robots...")
        if leader_robot and leader_robot._sock_req is not None : # Check if connected
            leader_robot.disconnect()
            print("Leader robot disconnected.")
        if follower_robot and follower_robot._sock_req is not None: # Check if connected
            follower_robot.disconnect()
            print("Follower robot disconnected.")

if __name__ == "__main__":
    # Note: This example will likely fail if no server is running at the specified addresses.
    # It's intended to show the structure of client-side code.
    # To run this, you would need a server that can handle requests for MsgID.GET_STATE_REQ
    # and MsgID.QUERY_STATE_REQ at the configured IPs/ports.
    # The current `test_robot_server.py` could be adapted or run twice on different ports
    # if it were modified to not exit immediately and to handle multiple clients or robot instances.
    print("Leader-Follower Example")
    print("This script attempts to connect to two (simulated/real) robot servers.")
    print(f"Ensure servers are running at {LEADER_ROBOT_IP}:{LEADER_ROBOT_PORT} (Leader) "
          f"and {FOLLOWER_ROBOT_IP}:{FOLLOWER_ROBOT_PORT} (Follower).")
    print("If using the provided test server, you might need to run two instances on these different ports.")

    run_leader_follower_example()
