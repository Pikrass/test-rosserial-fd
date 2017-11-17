## Compiling

First checkout the submodules:
```bash
$ git submodule update --init
```

rosserial needs a few tweaks to work correctly with the simulator and some
tests. Apply the provided patch:
```bash
$ (cd catkin_ws/src/rosserial/ && patch -p1 < ../../../rosserial.patch)
```

Then run `make` in this directory. You may have to install some packets first
(especially for simavr).


## Running the tests

Inside simulator/, run:
```bash
$ ./launch
```

In another terminal, after sourcing ROS and catkin setup scripts, run:
- for testing the python server
```bash
$ roslaunch test test-python.launch
```
- for testing the cpp server
```bash
$ roslaunch test test-cpp.launch
```

Wait until test_node exits cleanly. If it doesn't after a minute or so,
something may have gone wrong.

After killing roslaunch, the results will be in:
$HOME/.ros/log/latest/test_node-2-stdout.log


## Debugging

To debug rosserial_server, add `launch-prefix="screen -d -m gdb --args"` to the
node in the launch file. Connect to the screen session after running roslaunch
with:
```bash
$ screen -r
```

To debug the client, run `./launch -g` instead of `./launch` inside simulator/.
Then launch gdb:
```bash
$ avr-gdb ../arduino/test/build*/test.elf
> source gdbinit
```
