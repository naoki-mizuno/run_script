# run_script

A node to run scripts so that you can put in your launch file.


## Usage (from ROS parameters)

Parameters you can set:

```
shell: bash
commands:
  - some
  - commands
  - to
  - execute
# How long (in seconds) to wait before running the first command
delay: 0
# How long (in seconds) to wait between each command
interval 0
```

```
$ rosrun run_script run.py _commands:='[echo "hello", date]'
hello
Mon Mar 26 20:51:06 JST 2018
```

The `commands` parameter can be a string or a list, in which case each element
will be executed. The commands are executed by default via `bash -c COMMAND`,
unless an empty string or `nil` is specified for the `shell` parameter.

```
rr run_script run.py _commands:='echo "hello"' _shell:=nil
Traceback (most recent call last):
  File "/home/naoki/ros/workspaces/lunar/common/src/run_script/nodes/run.py", line 31, in <module>
    subprocess.call(command)
  File "/usr/lib/python2.7/subprocess.py", line 523, in call
    return Popen(*popenargs, **kwargs).wait()
  File "/usr/lib/python2.7/subprocess.py", line 711, in __init__
    errread, errwrite)
  File "/usr/lib/python2.7/subprocess.py", line 1343, in _execute_child
    raise child_exception
OSError: [Errno 2] No such file or directory
```

The error occurs because it tries to execute a command (file) with the name
`echo "hello"`.

Another example using a YAML file:

```
shell: zsh
commands:
  - ls
  - date
  - echo 'foo'
  # Built-in print is only available in zsh
  - print -l ~/*
```

```
$ rosparam load param.yaml run_script
$ rosrun run_script run.py
```

## Usage (from arguments)

`~/my_script.sh`:

```
date
echo $PWD
```

Running:

```
$ rosrun run_script run.py ~/my_script.sh
hello
Mon Mar 26 20:52:14 JST 2018
```

When both command line arguments and ROS parameters are provided, the ROS
parameters are prioritized.


## Usage (from a launch file)

```xml
<?xml version="1.0"?>
<launch>
  <include file="$(find run_script)/launch/run.launch" >
    <arg name="command" value="date" />
  </include>
</launch>
```

## License

MIT


## Author

Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
