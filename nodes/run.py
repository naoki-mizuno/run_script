#!/usr/bin/env python

import rospy

import subprocess
import sys


rospy.init_node('run_script')

shell = rospy.get_param('~shell', 'bash')
commands = rospy.get_param('~commands', sys.argv[1:])
delay = rospy.get_param('~delay', 0)
interval = rospy.get_param('~interval', 0)

if type(commands) is not list:
    commands = [commands]

rospy.sleep(delay)
first_iteration = True
for command in commands:
    if not first_iteration:
        rospy.sleep(interval)
    first_iteration = False

    if type(command) is not list:
        command = [command]

    rospy.logdebug('Running: ' + ' '.join(command))
    # Execute from a shell, for example, bash -c 'echo "foo"'
    if shell is not None and shell != '':
        command = [shell, '-c'] + command
    subprocess.call(command)

