# run_script

A node to run scripts so that you can put in your launch file. Say you
want to run `~/some_script.bash` with a certain delay. You will be
able to do the following:

```xml
<node name="run_script" pkg="run_script" type="run.py" >
  <param name="commands" value="~/some_script.bash" />
  <param name="delay" value="5" />
</node>
```


## Usage (from ROS parameters)

Here is an example of the parameters you can set:

```yaml
shell: bash
commands:
  - some
  - commands
  - to
  - execute
# How long (in seconds) to wait before running the first command
delay: 0
# How long (in seconds) to wait between each command
interval: 0
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
rosrun run_script run.py _commands:='echo "hello"' _shell:=nil
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

```yaml
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

An example of running a script from a launch file:

```xml
<?xml version="1.0"?>
<launch>
  <node name="run_script" pkg="run_script" type="run.py" >
    <param name="command" value="echo 'hello, world!'" />
  </node>
</launch>
```

When both command line arguments and ROS parameters are provided, the ROS
parameters are prioritized. That is, if the following launch file is used:

```xml
<?xml version="1.0"?>
<launch>
  <node name="run_script" pkg="run_script" type="run.py" args="date" >
    <param name="commands" value="echo 'hello, world!'" />
  </node>
</launch>
```

`echo 'hello, world!'` is executed.


## Usage (from the bundled launch file)

There is a launch file that is provided with the package, which you can use to
execute **one** command (but you can execute multiple commands if you really
want to by using `&&` and/or `;`). Note that the `arg` is the singular
`command` and not `commands`.

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
