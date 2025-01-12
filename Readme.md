# LTTng Tracing examples with ROS2
This repository contains many examples, demonstrating the features provided by lttng when used in the context of ROS2.

## Recording of traces
To start the recording of traces with LTTNG, you first need to create a session.
```bash
lttng-sessiond --daemonize
```
Once the session is created, you can then start exploring the list of tracepoints that are available for your program. Do the following command to list the available user space tracepoints. 

**Note that the tracepoints you see after running this command doesn't necesary indicate that there is a call to record the tracepoint within the code. It just indicates that the definitions and declarations of the tracepoints have been found**
```bash
lttng list --userspace
```
Then, create a recording session.
```bash
# The basic syntax is `lttng create <session_name>`
lttng create example1  # this creates the session named example1
```
To start recording the events, you need to add a recording event rule that matches the tracepoint events that you want to capture.

For example, in the hello_world.cpp, the tracepoint used is hello_world:my_first_tracepoint. To record that, we can add a rule directly matching that tracepoint.
```bash
lttng enable-event --userspace hello_world:my_first_tracepoint
```
Or, we can use bash wildcards to match all tracepoint events matching the rule.
```bash
# This will match all tracepoint events beginning with hello"
lttng enable-event --userspace "hello*"
```
Once the rules have been created, you can start recording.
```bash
lttng start
```
When the application is running, the program will execute the instrumentation points, which will be recorded in the session.

For example, to run the "hello_world" example in this repo, 
```bash
ros2 run tracing_example hello_world
```

To observe the tracepoint events recorded, you need to first stop the session. `lttng start` will resume the stopped session.
```bash
lttng stop
```
When you want to finish a session, you need to destroy the session. Once session is destroyed, it will no longer record. Subsequent call to `lttng start` will simply create a new session.
```bash
lttng destroy
```

## Analysis of the traces
By default, the traces would be saved to /home/tracer/lttng-traces. To verify the exact events that has been recorded, you can use babeltrace that is preinstalled in the image.
```bash
# syntax is babeltrace <path_to_trace>
babeltrace /home/tracer/lttng-traces/example1-20250112-030643   # folder name is example1 + the datestamp of the session
```
To do more detailed analysis, you can use the GUI Trace Compass instead for better convenience.
https://eclipse.dev/tracecompass/

## Function call stack profiling
LTTNG also supports the tracing of function entries and exits, allowing for detailed analysis of the call stack per thread of the program over time. There are a few steps needed to enable this functionality.

First, the application to be profiled needs to be recompiled to add the instrumentation points automatically. In the ROS2 framework, this can be done by supplying addition C++ flags to the build.
```bash
# Adds the flags "-g -finstrument-functions"
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-g -finstrument-functions"
```
Once the application has been built, the liblttng-ust-cyg-profile library needs to be preloaded when running the application, by setting the environment variable.
```bash
# Change the LD_PRELOAD environment variable to point to the library
export LD_PRELOAD=/usr/lib/x86_84-linux-gnu/liblttng-ust-cyg-profile.so
```
Next, you need to configure the tracing session to record the relevant events.
```bash
# Create the session
lttng create session
```
```bash
# Add recording rules for the function tracing. Simple way is to trace everything
lttng enable-event -u -a
```
```bash
# Add the relevant context fields
lttng add-context -u -t vpid -t vtid -t procname
```
Now, you can start the session.
```bash
lttng start
```
Then, run the application as normal. For example, for the `hello_world` application, the command needed is shown below.
```bash
ros2 run tracing_example hello_world
```
To stop the application,
```bash
# Pauses the session
lttng stop
```
```bash
# Completes the session
lttng destroy
```
At this point, the traces have been recorded. However, the symbols recorded in the traces refer to the actual function addresses, which are not very useful by themselves. To get the function names, we need to get the mapping from addresses to function names.

Build the previous application with debug symbols.
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
Then, create the mapping using `nm`. If using C++, add the flag `--demangle`
```bash
# Use nm on the executable path to get the symbol mapping
# Example here shows the command for the hello_world executable
nm --demangle /home/tracer/trace_ws/install/tracing_example/lib/tracing_example/hello_world > mapping.txt
```
The created `mapping.txt` can be added into Trace Compass during the analysis to provide the mapping.

## Special Notes
- Other C++ flags can be added when instrumenting functions to make the analysis easier. 
    - `-fno-omit-frame-pointer` forces storage of stack frame pointers during function calls.
    - `finstrument-functions-exclude-file-list` allows specifying the files not to be instrumented. For example, the standard library can be excluded from instrumentation to reduce clutter. Following command shows how to use it
    ```bash
    # This command excludes the standard library
    # -finstrument-functions must also be added
    colcon build --cmake-args -DCMAKE_CXX_FLAGS="-finstrument-functions -finstrument-functions-exclude-file-list=/usr/include"
    ```


## References
This guide has been created using the help of many resources, links provided below.
- [Official LTTng documentation](https://lttng.org/docs/v2.13/)
- [TraceCompass - LTTng-UST Analyses](https://archive.eclipse.org/tracecompass/doc/stable/org.eclipse.tracecompass.doc.user/LTTng-UST-Analyses.html)
