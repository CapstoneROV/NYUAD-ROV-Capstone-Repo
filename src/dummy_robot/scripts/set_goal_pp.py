#!/usr/bin/env python3
import pexpect

def main():
    # Launch MoveIt Commander CLI
    child = pexpect.spawn('rosrun moveit_commander moveit_commander_cmdline.py')
    child.expect('>')

    # Connect to the specified robot group
    child.sendline('use base')
    child.expect('>')

    # Get the current state of the group
    child.sendline('current')
    child.expect('>')

    # Record the current state under a specific name
    child.sendline('rec c')
    child.expect('>')

    # Prompt the user to input the 7 values for the goal
    goal_values = input("Please input the 7 values for the goal (x y z qx qy qz qw): ").split()

    # Set the goal state using the recorded state as a base
    child.sendline('goal = c')
    child.expect('>')

    # Update the goal state with the input values
    for i, value in enumerate(goal_values):
        child.sendline(f'goal[{i}] = {value}')
        child.expect('>')

    # Plan the motion to the goal state
    child.sendline('plan goal')
    child.expect('Planning to goal: Succeeded')
    child.expect('>')

    # Execute the computed motion plan
    child.sendline('execute')
    child.expect('Motion execution completed')
    child.expect('>')

    # Close the MoveIt Commander CLI
    child.sendline('quit')
    child.expect(pexpect.EOF)

if __name__ == '__main__':
    main()
