
# ROS IN 5 DAYS 
Unit 1: Basic Concepts


Estimated time to completion: 1'5 hours 

Simulated robot: Turtlebot 

What will you learn with this unit?

How to structure and launch ROS programs (packages and launch files)
How to create basic ROS programs (C++ based)
Basic ROS concepts: Nodes, Parameter Server, Environment Variables, Roscore


## What is ROS?
This is probably the question that has brought you all here. Well, let me tell you that you are still not prepared to understand the answer to this question, so... let's get some work done first.

Move a Robot with ROS
On the right corner of the screen, you have your first simulated robot: the Turtlebot 2 robot against a large wall.



Let's move that robot!

How can you move the Turtlebot?
The easiest way is by executing an existing ROS program to control the robot. A ROS program is executed by using some special files called launch files.

Since a previously-made ROS program already exists that allows you to move the robot using the keyboard, let's launch that ROS program to teleoperate the robot.
Example 1.1

Execute the following command in WebShell number #1

Execute in WebShell #1


roslaunch turtlebot_teleop keyboard_teleop.launch
roslaunch turtlebot_teleop keyboard_teleop.launch
WebShell #1 Output

# Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .
​
q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
​
CTRL-C to quit
Now, you can use the keys indicated in the WebShell Output in order to move the robot around. The basic keys are the following:

	
Move forward

	
Move backward

	
Turn left

	
Turn right

	
Stop

	
Increase / Decrease Speed

Try it!! When you're done, you can Ctrl+C to stop the execution of the program.

roslaunch is the command used to launch a ROS program. Its structure goes as follows:

roslaunch <package_name> <launch_file>
roslaunch <package_name> <launch_file>
As you can see, that command has two parameters: the first one is the name of the package that contains the launch file, and the second one is the name of the launch file itself (which is stored inside the package).

END Example 1.1


Now... what's a package?
ROS uses packages to organize its programs. You can think of a package as all the files that a specific ROS program contains; all its cpp files, python files, configuration files, compilation files, launch files, and parameters files.
All those files in the package are organized with the following structure:

launch folder: Contains launch files
src folder: Source files (cpp, python)
CMakeLists.txt: List of cmake rules for compilation
package.xml: Package information and dependencies
To go to any ROS package, ROS gives you a command named roscd. When typing:

roscd <package_name>
roscd <package_name>
It will take you to the path where the package package_name is located.

Example 1.2


Go to WebShell #1, navigate to the turtlebot_teleop package, and check that it has that structure.

Execute in WebShell #1


roscd turtlebot_teleop
ls


Every ROS program that you want to execute is organized in a package.
Every ROS program that you create will have to be organized in a package.
Packages are the main organization system of ROS programs.

END Example 1.2


And... what's a launch file?
We've seen that ROS uses launch files in order to execute programs. But... how do they work? Let's have a look.

Example 1.3


Open the launch folder inside the turtlebot_teleop package and check the keyboard_teleop.launch file.

Execute in WebShell #1


roscd turtlebot_teleop
cd launch
cat keyboard_teleop_cpp.launch
WebShell #1 Output

<launch>
  <!-- turtlebot_teleop_key already has its own built in velocity smoother -->
  <node pkg="turtlebot_teleop" type="turtlebot_teleop_key" name="turtlebot_teleop_keyboard"  output="screen">
    <param name="scale_linear" value="0.5" type="double"/>
    <param name="scale_angular" value="1.5" type="double"/>
    <remap from="turtlebot_teleop_keyboard/cmd_vel" to="cmd_vel_mux/input/teleop"/>
  </node>
</launch>
In the launch file, you have some extra tags for setting parameters and remaps. For now, don't worry about those tags and focus on the node tag.
All launch files are contained within a <launch> tag. Inside that tag, you can see a <node> tag, where we specify the following parameters:

pkg="package_name" # Name of the package that contains the code of the ROS program to execute
type="cpp_executable_name" # Name of the cpp executable file that we want to execute
name="node_name" # Name of the ROS node that will launch our C++ file
output="type_of_output" # Through which channel you will print the output of the program
END Example 1.3

Create a package
Until now we've been checking the structure of an already-built package... but now, let's create one ourselves.

When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The catkin workspace is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called catkin_ws.

Example 1.4


Go to the catkin_ws in your webshell.

In order to do this, type roscd in the shell. You'll see that you are thrown to a catkin_ws/devel directory. Since you want to go to the workspace, just type cd .. to move up 1 directory. You must end up here in the /home/user/catkin_ws.
Execute in WebShell #1

roscd
cd ..
pwd
WebShell #1 Output

user ~ $ pwd
/home/user/catkin_ws
Inside this workspace, there is a directory called src. This folder will contain all the packages created. Every time you want to create a package, you have to be in this directory (catkin_ws/src).Type in your web shell cd src in order to move to the source directory.

Execute in WebShell #1

cd src
Now we are ready to create our first package! In order to create a package, type in your webshell:

Execute in WebShell #1

catkin_create_pkg my_package roscpp
This will create inside our "src" directory a new package with some files in it. We'll check this later. Now, let's see how this command is built:

catkin_create_pkg <package_name> <package_dependecies>
The package_name is the name of the package you want to create, and the package_dependencies are the names of other ROS packages that your package depends on.

END Example 1.4

Example 1.5

In order to check that our package has been created successfully, we can use some ROS commands related to packages. For example, let's type:

Execute in WebShell #1

rospack list
rospack list | grep my_package
roscd my_package 
rospack list: Gives you a list with all of the packages in your ROS system.
rospack list | grep my_package: Filters, from all of the packages located in the ROS system, the package named my_package.
roscd my_package: Takes you to the location in the Hard Drive of the package, named my_package. 

You can also see the package created and its contents by just opening it through the IDE (similar to {Figure 1.1})


Fig.1.1 - IDE created package my_package
END Example 1.5


Compile a package
When you create a package, you will usually need to compile it in order to make it work. The command used by ROS to compile is the next one:

catkin_make
This command will compile your whole src directory, and it needs to be issued in your catkin_ws directory in order to work. This is MANDATORY. If you try to compile from another directory, it won't work.

Example 1.6

Go to your catkin_ws directory and compile your source folder. You can do this by typing:

Execute in WebShell #1

roscd; cd ..
catkin_make
Sometimes (for example, in large projects) you will not want to compile all of your packages, but just the one(s) where you've made changes. You can do this with the following command:

catkin_make --only-pkg-with-deps <package_name>
This command will only compile the packages specified and its dependencies.

Try to compile your package named my_package with this command.

Execute in WebShell #1

catkin_make --only-pkg-with-deps my_package
END Example 1.6


My first ROS program
At this point, you should have your first package created... but now you need to do something with it! Let's do our first ROS program!

Example 1.7

1- Create in the src directory in my_package a C++ file that will be executed. For this exercise, just copy this simple C++ code simple.cpp. You can create it directly by RIGHT clicking on the IDE on the src directory of your package, selecting New File, and writing the name of the file on the box that will appear.



A new Tab should have appeared on the IDE with empty content. Then, copy the content of simple.cpp into the new file. Finally, press Ctrl-S to save your file with the changes. The Tab in the IDE will go from Green to no color (see pictures below).





2- Create a launch directory inside the package named my_package {Example 1.4}.

Execute in WebShell #1

roscd my_package
mkdir launch
You can also create it through the IDE.

3- Create a new launch file inside the launch directory.

Execute in WebShell #1

touch launch/my_package_launch_file.launch
You can also create it through the IDE.

4- Fill this launch file as we've previously seen in this course {Example 1.3}.

HINT: You can copy from the turtlebot_teleop package, the keyboard_teleop.launch file and modify it. If you do so, remove the param and remap tags and leave only the node tag, because you don't need those parameters.

The final launch should be something similar to this: my_package_launch_file.launch

5- Modify the CMakeLists.txt file in order to generate an executable from the C++ file you have just created.

Note: This is something that is required when working in ROS with C++. When you finish this Exercise, you'll learn more about this subject. For now, just follow the instructions below.

In the Build section of your CMakeLists.txt file, add the following lines:

add_executable(simple src/simple.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )


HINT: If you have a look at the file, you'll see that those lines are already in the file, but they are commented. If you feel so, you can uncomment those lines and modify them like the ones we provide above, instead of simply adding them to the end of the section.

6- Compile your package as explained previously.

Execute in WebShell #1

roscd;
cd ..;
catkin_make
If everything goes fine, you should get something like this as output:



7- Finally, execute the roslaunch command in the WebShell in order to launch your program.

Execute in WebShell #1

roslaunch my_package my_package_launch_file.launch
END Example 1.7

Expected Result for Example 1.6

You should see Leia's quote among the output of the roslaunch command.

WebShell #1 Output

user catkin_ws $ roslaunch my_package my_package_launch_file.launch
logging to /home/user/.ros/log/d29014ac-911c-11e6-b306-02f9ff83faab/roslaunch-ip-172-31-30-5-28204.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
​
started roslaunch server http://ip-172-31-30-5:40504/
​
SUMMARY
========
​
PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20
​
NODES
  /
    ObiWan (my_package/simple)
​
ROS_MASTER_URI=http://localhost:11311
​
core service [/rosout] found
process[ObiWan-1]: started with pid [28228]
[ INFO] [1515028076.948011193]: Help me Obi-Wan Kenobi, you're my only hope
[ObiWan-1] process has finished cleanly
log file: /home/user/.ros/log/d29014ac-911c-11e6-b306-02f9ff83faab/ObiWan-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor complete
done
END Expected Result for Example 1.6

Sometimes ROS won't detect a new package when you have just created it, so you won't be able to do a roslaunch. In this case, you can force ROS to do a refresh of its package list with the command:

Execute in WebShell #1

rospack profile
C++ Program {1.1a-cpp}: simple.cpp

#include <ros/ros.h>
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "ObiWan");
    ros::NodeHandle nh;
    ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope");
    ros::spinOnce();
    return 0;
}
You may be wondering what this whole code means, right? Well, let's explain it line by line:

#include <ros/ros.h>
// Here we are including all the headers necessary to use the most common public pieces of the ROS system.
// Always we create a new C++ file, we will need to add this include.
​
int main(int argc, char** argv) { // We start the main C++ program
​
    ros::init(argc, argv, "ObiWan"); // We initiate a ROS node called ObiWan
    ros::NodeHandle nh; // We create a handler for the node. This handler will actually do the initialization of the                           // node
    ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope"); // This is the same as a print in ROS
    ros::spinOnce(); // Calling ros::spinOnce() here is not necessary for this simple program, because we are not                          // receiving any callbacks. However, if you were to add a subscription into this application, and                      // did not have ros::spinOnce() here, your callbacks would never get called. So, add it for good 
                     // measure.
    return 0; // We end our program
}
NOTE: If you create your C++ file from the shell, it may happen that it's created without execution permissions. If this happens, ROS won't be able to find it. If this is your case, you can give execution permissions to the file by typing the next command: chmod +x name_of_the_file.cpp
END C++ Program {1.1-cpp}: simple.py


Launch File {1.1-l}: my_package_launch_file.launch

You should have something similar to this in your my_package_launch_file.launch:

Note: Keep in mind that in the example below, the C++ executable name in the attribute type is named simple. So, if you have named your C++ executable with a different name, this will be different.

<launch>
    <!-- My Package launch file -->
    <node pkg="my_package" type="simple" name="ObiWan"  output="screen">
    </node>
</launch>
END Launch File {1.1-l}: my_package_launch_file.launch

Modifying the CMakeLists.txt file
When coding with C++, it will be necessary to create binaries(executables) of your programs in order to be able to execute them. For that, you will need to modify the CMakeLists.txt file of your package, in order to indicate that you want to create an executable of your C++ file.

To do this, you need to add some lines into your CMakeLists.txt file. In fact, these lines are already in the file, but they are commented. You can also find them, and uncomment them. Whatever you want.

In the previous Exercise, you had the following lines:

add_executable(simple src/simple.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
But... what do this lines of code exactly do? Well, basically they do the following:

add_executable(simple src/simple.cpp)
This line generates an executable from the simple.cpp file, which is in the src folder of your package. This executable will be placed by default into the package directory of your devel space, which is located by default at ~/catkin_ws/devel/lib/.

add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
This line adds all the cmake target dependencies of the executable. It's used basically to allow CMake to correctly compile a package, making sure all the dependencies are in place.

target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
This line specifies the libraries to use when linking a given target. For this case, it indicates to use the catkin libraries when linking to the executable you have created.

If you are interested, you can read more about the CMakeLists.txt file here: http://wiki.ros.org/catkin/CMakeLists.txt

ROS Nodes
You've initiated a node in the previous code but... what's a node? ROS nodes are basically programs made in ROS. The ROS command to see what nodes are actually running in a computer is:

rosnode list
Example 1.8

Type this command in a new shell and look for the node you've just initiated (ObiWan).

Execute in WebShell #1

rosnode list
You can't find it? I know you can't. That's because the node is killed when the C++ program ends.
Let's change that.

Update your C++ file simple.cpp with the following code:

C++ Program {1.1b-cpp}: simple_loop.cpp

#include <ros/ros.h>
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "ObiWan");
    ros::NodeHandle nh;
    ros::Rate loop_rate(2); // We create a Rate object of 2Hz
    
    while (ros::ok()) // Endless loop until Ctrl + C
    {
        ROS_INFO("Help me Obi-Wan Kenobi, you're my only hope");
        ros::spinOnce();
        loop_rate.sleep(); // We sleep the needed time to maintain the Rate fixed above
    }
    
    return 0;
}
​
// This program creates an endless loop that repeats itself 2 times per second (2Hz) until somebody presses Ctrl + C
// in the Shell
END C++ Program {1.1b-cpp}: simple_loop.cpp

Launch your program again using the roslaunch command.

Execute in WebShell #1

roslaunch my_package my_package_launch_file.launch
Now try again in another Web Shell:

Execute in WebShell #2

rosnode list
Can you now see your node?

WebShell #1 Output

user ~ $ rosnode list
/ObiWan
/cmd_vel_mux
/gazebo
/mobile_base_nodelet_manager
/robot_state_publisher
/rosout
In order to see information about our node, we can use the next command:

rosnode info /ObiWan
This command will show us information about all the connections that our Node has.

Execute in WebShell #1

rosnode info /ObiWan
WebShell #1 Output

user ~ $ rosnode info /ObiWan
--------------------------------------------------------------------------------
Node [/ObiWan]
Publications:
 * /rosout [rosgraph_msgs/Log]
​
Subscriptions:
 * /clock [rosgraph_msgs/Clock]
​
Services:
 * /ObiWan/set_logger_level
 * /ObiWan/get_loggers
​
​
contacting node http://ip-172-31-30-5:58680/ ...
Pid: 1215
Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo (http://ip-172-31-30-5:46415/)
    * direction: inbound
    * transport: TCPROS
For now, don't worry about the output of the command. You will understand more while going through the next tutorial.

END Example 1.8

Parameter Server
A Parameter Server is a dictionary that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.

To get a list of these parameters, you can type:

rosparam list
To get a value of a particular parameter, you can type:

rosparam get <parameter_name>
And to set a value to a parameter, you can type:

rosparam set <parameter_name> <value>
Example 1.9

To get the value of the '/camera/imager_rate' parameter, and change it to '4.0,' you will have to do the following:

Execute in WebShell #1

rosparam get /camera/imager_rate
rosparam set /camera/imager_rate 4.0
rosparam get /camera/imager_rate
END Example 1.9

You can create and delete new parameters for your own use, but do not worry about this right now. You will learn more about this in more advanced tutorials

Roscore
In order to have all of this working, we need to have a roscore running. The roscore is the main process that manages all of the ROS system. You always need to have a roscore running in order to work with ROS. The command that launches a roscore is:

roscore

Fig.1.2 - ROS Core Diagram
NOTE: At the platform you are using for this course, when you enter a course it automatically launches a roscore for you, so you don't need to launch one.
Environment Variables
ROS uses a set of Linux system environment variables in order to work properly. You can check these variables by typing:

export | grep ROS
NOTE 1: Depending on your computer, it could happen that you can't type the | symbol directly in your webshell. If that's the case, just copy/paste the command by RIGHT-CLICKING on the WebShell and select Paste from Browser. This feature will allow you to write anything on your webshell, no matter what your computer configuration is.
user ~ $ export | grep ROS
declare -x ROSLISP_PACKAGE_DIRECTORIES="/home/user/catkin_ws/devel/share/common-lisp"
declare -x ROS_DISTRO="indigo"
declare -x ROS_ETC_DIR="/opt/ros/indigo/etc/ros"
declare -x ROS_MASTER_URI="http://localhost:11311"
declare -x ROS_PACKAGE_PATH="/home/user/catkin_ws/src:/opt/ros/indigo/share:/opt/ros/indigo/stacks"
declare -x ROS_ROOT="/opt/ros/indigo/share/ros"
The most important variables are the ROS_MASTER_URI and the ROS_PACKAGE_PATH.

ROS_MASTER_URI -> Contains the url where the ROS Core is being executed. Usually, your own computer (localhost).
ROS_PACKAGE_PATH -> Contains the paths in your Hard Drive where ROS has packages in it.
NOTE 2: At the platform you are using for this course, we have created an alias to display the environment variables of ROS. This alias is rosenv. By typing this on your shell, you'll get a list of ROS environment variables. It is important that you know that this is not an official ROS command, so you can only use it while working on this platform.
So now... what is ROS?
ROS is basically the framework that allows us to do all that we showed along this chapter. It provides the background to manage all these processes and communications between them... and much, much more!! In this tutorial you've just scratched the surface of ROS, the basic concepts. ROS is an extremely powerful tool. If you dive into our courses you'll learn much more about ROS and you'll find yourself able to do almost anything with your robots!

Additional material to learn more:
ROS Packages: http://wiki.ros.org/Packages

Ros Nodes: http://wiki.ros.org/Nodes

Parameter Server: http://wiki.ros.org/Parameter%20Server

Roscore: http://wiki.ros.org/roscore

ROS Environment Variables: http://wiki.ros.org/ROS/EnvironmentVariables



ROS IN 5 DAYS
Unit 2: Topics


Estimated time to completion: 2.5 hours 

Simulation: Turtlebot 

What will you learn with this unit?

What are ROS topics and how to manage them
What is a publisher and how to create one
What are topic messages and how they work


Part 1: Publisher
Exercise 2.1

Create a new package named topic_publisher_pkg. When creating the package, add as dependencies roscpp and std_msgs.

Inside the src folder of the package, create a new file named simple_topic_publisher.cpp. Inside this file, copy the contents of simple_topic_publisher.cpp

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

END Exercise 2.1

Data for Excercice 2.1


1.- In order to do this exercise, you can simply follow the same steps you made in the previous Chapter. It is almost the same.

2.- Remember, in order to create a package with roscpp and std_msgs as dependencies, you should use a command like the below one:

catkin_create_pkg topic_publisher_pkg roscpp std_msgs
3.- The lines to add into the CmakeLists.txt file could be something like this:

add_executable(simple_topic_publisher src/simple_topic_publisher.cpp)
add_dependencies(simple_topic_publisher ${simple_topic_publisher_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_topic_publisher
   ${catkin_LIBRARIES}
 )
C++ Program {2.1}: simple_topic_publisher.cpp

#include <ros/ros.h>
#include <std_msgs/Int32.h>
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000);
    ros::Rate loop_rate(2);
    
    std_msgs::Int32 count;
    count.data = 0;
    
    while (ros::ok())
    {
        pub.publish(count);
        ros::spinOnce();
        loop_rate.sleep();
        ++count.data;
    }
    
    return 0;
}
END C++ Program {2.1}: simple_topic_publisher.cpp

Nothing happens? Well... that's not actually true! You have just created a topic named /counter, and published through it as an integer that increases indefinitely. Let's check some things.

A topic is like a pipe. Nodes use topics to publish information for other nodes so that they can communicate. 
You can find out, at any time, the number of topics in the system by doing a rostopic list. You can also check for a specific topic.

On your webshell, type rostopic list and check for a topic named '/counter'.

Execute in WebShell #1

rostopic list | grep  '/counter'
WebShell #1 Output

user ~ $ rostopic list | grep '/counter'
/counter
Here, you have just listed all of the topics running right now and filtered with the grep command the ones that contain the word /counter. If it appears, then the topic is running as it should.

You can request information about a topic by doing rostopic info <name_of_topic>.

Now, type rostopic info /counter.

Execute in WebShell #1

rostopic info /counter
WebShell #1 Output

user ~ $ rostopic info /counter
Type: std_msgs/Int32
​
Publishers:
 * /topic_publisher (http://ip-172-31-16-133:47971/)
​
Subscribers: None
The output indicates the type of information published (std_msgs/Int32), the node that is publishing (/topic_publisher), and if there is a node listening to that info (None in this case).

Now, type rostopic echo /counter and check the output of the topic in realtime.

Execute in WebShell #1


rostopic echo /counter
You should see a succession of consecutive numbers, similar to the following:

WebShell #1 Output

rostopic echo /counter
data:
985
---
data:
986
---
data:
987
---
data:
988
---
Ok, so... what has just happened? Let's explain it in more detail. First, let's crumble the code we've executed. You can check the comments in the code below explaining what each line of the code does:

#include <ros/ros.h>
#include <std_msgs/Int32.h>
// Import all the necessary ROS libraries and import the Int32 message from the std_msgs package
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_publisher"); // Initiate a Node named 'topic_publisher'
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::Int32>("counter", 1000); // Create a Publisher object, that will                                                                                // publish on the /counter topic messages
                                                                         // of type Int32
    ros::Rate loop_rate(2); // Set a publish rate of 2 Hz
    
    std_msgs::Int32 count; // Create a variable of type Int32
    count.data = 0; // Initialize 'count' variable
    
    while (ros::ok()) // Create a loop that will go until someone stops the program execution
    {
        pub.publish(count); // Publish the message within the 'count' variable
        ros::spinOnce();
        loop_rate.sleep(); // Make sure the publish rate maintains at 2 Hz
        ++count.data; // Increment 'count' variable
    }
    
    return 0;
}
So basically, what this code does is to initiate a node and create a publisher that keeps publishing into the '/counter' topic a sequence of consecutive integers. Summarizing:

A publisher is a node that keeps publishing a message into a topic. So now... what's a topic?

A topic is a channel that acts as a pipe, where other ROS nodes can either publish or read information. Let's now see some commands related to topics (some of them you've already used).

To get a list of available topics in a ROS system, you have to use the next command:

rostopic list
To read the information that is being published in a topic, use the next command:

rostopic echo <topic_name>
This command will start printing all of the information that is being published into the topic, which sometimes (ie: when there's a massive amount of information, or when messages have a very large structure) can be annoying. In this case, you can read just the last message published into a topic with the next command:

rostopic echo <topic_name> -n1
To get information about a certain topic, use the next command:

rostopic info <topic_name>
Finally, you can check the different options that rostopic command has by using the next command:

rostopic -h
IMPORTANT NOTE

When you have finished with this section of the Notebook, make sure to STOP the previously executed code by selecting the cell with the code and clicking on the Interrupt kernel button at the top right corner of the Notebook. This is very important for doing the Next Unit properly.

IMPORTANT NOTE

Messages
As you may have noticed, topics handle information through messages. There are many different types of messages.

In the case of the code you executed before, the message type was an std_msgs/Int32, but ROS provides a lot of different messages. You can even create your own messages, but it is recommended to use ROS default messages when its possible.

Messages are defined in .msg files, which are located inside a msg directory of a package.

To get information about a message, you use the next command:

rosmsg show <message>
Example 2.1

For example, let's try to get information about the std_msgs/Int32 message. Type the following command and check the output.

Execute in WebShell #1

rosmsg show std_msgs/Int32
WebShell #1 Output

user ~ $ rosmsg show std_msgs/Int32
[std_msgs/Int32]:
int32 data
In this case, the Int32 message has only one variable named data of type int32. This Int32 message comes from the package std_msgs, and you can find it in its msg directory. If you want, you can have a look at the Int32.msg file by executing the following command:

roscd std_msgs/msg/
END Example 2.1

Now you're ready to create your own publisher and make the robot move, so let's go for it!

Exercise 2.2


Modify the code you used previously so that now it publishes data to the /cmd_vel topic.
Compile again your package.
Launch the program and check that the robot moves.
Data for Excercice 2.2


1.- The /cmd_vel topic is the topic used to move the robot. Do a rostopic info /cmd_vel in order to get information about this topic, and identify the message it uses. You have to modify the code to use that message.

2.- In order to fill the Twist message, you need to create an instance of the message. In C++, this is done like this: geometry_msgs::Twist var;

3.- In order to know the structure of the Twist messages, you need to use the rosmsg show command, with the type of the message used by the topic /cmd_vel.

4.- In this case, the robot uses a differential drive plugin to move. That is, the robot can only move linearly in the x axis, or rotationaly in the angular z axis. This means that the only values that you need to fill in the Twist message are the linear x and the angular z.



5.- The magnitudes of the Twist message are in m/s, so it is recommended to use values between 0 and 1. For example, 0'5 m/s.

END Exercise 2.2

Solutions

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions for the Topics Part 1:Topics Part 1 Solutions



ROS IN 5 DAYS
Unit 2: Topics


Estimated time to completion: 2.5 hours 

What will you learn with this unit?

What is a Subscriber and how to create one
How to create your own message


Part 2: Subscriber
You've learned that a topic is a channel where nodes can either write or read information. You've also seen that you can write into a topic using a publisher, so you may be thinking that there should also be some kind of similar tool to read information from a topic. And you're right! That's called a subscriber. A subscriber is a node that reads information from a topic. Let's execute the next code:

Example 2.3

Create a new package named topic_subscriber_pkg. When creating the package, add as dependencies roscpp and std_msgs.

Inside the src folder of the package, create a new file named simple_topic_subscriber.cpp. Inside this file, copy the contents of simple_topic_subscriber.cpp

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

END Example 2.3

C++ Program {2.2}: simple_topic_subscriber.cpp

#include <ros/ros.h>
#include <std_msgs/Int32.h>
​
void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback);
    
    ros::spin();
    
    return 0;
}
END C++ Program {2.2}: simple_topic_subscriber.cpp


What's up? Nothing happened again? Well, that's not actually true... Let's do some checks.

Go to your webshell and type the following:

Execute in WebShell #1

rostopic echo /counter
You should see an output like this:

WebShell #1 Output

user ~ $ rostopic echo /counter
WARNING: no messages received and simulated time is active.
Is /clock being published?
And what does this mean? This means that nobody is publishing into the /counter topic, so there's no information to be read. Let's then publish something into the topic and see what happens. For that, let's introduce a new command:

rostopic pub <topic_name> <message_type> <value>
This command will publish the message you specify with the value you specify, in the topic you specify.

Open another webshell (leave the one with the rostopic echo opened) and type the next command:

Execute in WebShell #2

rostopic pub /counter std_msgs/Int32 5
Now check the output of the console where you did the rostopic echo again. You should see something like this: .

WebShell #1 Output

user ~ $ rostopic echo /counter
WARNING: no messages received and simulated time is active.
Is /clock being published?
data:
5
---
This means that the value you published has been received by your subscriber program (which prints the value on the screen).

Now check the output of the shell where you executed your subscriber code. You should now see something like this:



Before explaining everything with more detail, let's explain the code you executed.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
​
void counterCallback(const std_msgs::Int32::ConstPtr& msg) // Define a function called 'callback' that receives a                                                                // parameter named 'msg' 
{
  ROS_INFO("%d", msg->data); // Print the value 'data' inside the 'msg' parameter
}
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_subscriber"); // Initiate a Node called 'topic_subscriber'
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback); // Create a Subscriber object that will                                                                               // listen to the /counter topic and will
                                                                          // call the 'callback' function each time                                                                             // it reads something from the topic
    
    ros::spin(); // Create a loop that will keep the program in execution
    
    return 0;
}
So now, let's explain what has just happened. You've basically created a subscriber node that listens to the /counter topic, and each time it reads something, it calls a function that does a print of the msg. Initially, nothing happened since nobody was publishing into the /counter topic, but when you executed the rostopic pub command, you published a message into the /counter topic, so your subscriber has printed that number and you could also see that message in the rostopic echo output. Now everything makes sense, right?

Now let's do some exercises to put into practice what you've learned!

Exercise 2.4


Modify the previous code in order to print the odometry of the robot.
Data for Exercice 2.4


The odometry of the robot is published by the robot into the /odom topic.
You will need to figure out what message uses the /odom topic, and how the structure of this message is.
Remember to compile again your package in order to update your executable.
END Exercise 2.4

Solution Exercise 2.4

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit2 Topics Part2:Topics Part2 Solutions

END Solution Exercise 2.4

Exercise 2.5


Add to the {Exercice 2.4}, a C++ file that creates a publisher that indicates the age of the robot, to the previous package. 
For that, you'll need to create a new message called Age.msg. See the detailed description How to prepare CMakeLists.txt and package.xml for custom topic message compilation.
END Exercise 2.5

Solution Exercise 2.5

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit2 Topics Part2:Topics Part2 Solutions

END Solution Exercise 2.5

How to Prepare CMakeLists.txt and package.xml for Custom Topic Message Compilation
Now you may be wondering... in case I need to publish some data that is not an Int32, which type of message should I use? You can use all ROS defined (rosmsg list) messages. But, in case none fit your needs, you can create a new one.

In order to create a new message, you will need to do the following steps:

Create a directory named 'msg' inside your package
Inside this directory, create a file named Name_of_your_message.msg (more information down)
Modify CMakeLists.txt file (more information down)
Modify package.xml file (more information down)
Compile
Use in code
For example, let's create a message that indicates age, with years, months, and days.

1) Create a directory msg in your package.

roscd <package_name>
mkdir msg
2) The Age.msg file must contain this:

float32 years
float32 months
float32 days
3) In CMakeLists.txt

You will have to edit four functions inside CMakeLists.txt:

find_package()
add_message_files()
generate_messages()
catkin_package()
I. find_package()
This is where all the packages required to COMPILE the messages of the topics, services, and actions go. In package.xml, you have to state them as build_depend.

HINT 1: If you open the CMakeLists.txt file in your IDE, you'll see that almost all of the file is commented. This includes some of the lines you will have to modify. Instead of copying and pasting the lines below, find the equivalents in the file and uncomment them, and then add the parts that are missing.
find_package(catkin REQUIRED COMPONENTS
       roscpp
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)
II. add_message_files()
This function includes all of the messages of this package (in the msg folder) to be compiled. The file should look like this.

add_message_files(
      FILES
      Age.msg
    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO
III. generate_messages()
Here is where the packages needed for the messages compilation are imported.

generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO
IV. catkin_package()
State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as exec_depend.

catkin_package(
      CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here
)
Summarizing, this is the minimum expression of what is needed for the CMakaelist.txt to work:

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.

cmake_minimum_required(VERSION 2.8.3)
project(topic_ex)
​
​
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
​
add_message_files(
  FILES
  Age.msg
)
​
generate_messages(
  DEPENDENCIES
  std_msgs
)
​
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)
​
include_directories(
  ${catkin_INCLUDE_DIRS}
)
4) Modify package.xml

Just add these 2 lines to the package.xml file.

<build_depend>message_generation</build_depend>
​
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
This is the minimum expression of the package.xml

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.

<?xml version="1.0"?>
<package format="2">
  <name>topic_ex</name>
  <version>0.0.0</version>
  <description>The topic_ex package</description>
​
​
  <maintainer email="user@todo.todo">user</maintainer>
​
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
​
  <export>
​
  </export>
</package>
5) Now you have to compile the msgs. To do this, you have to type in a WebShell:

Execute in WebShell #1

roscd; cd ..
catkin_make
source devel/setup.bash
VERY IMPORTANT: When you compile new messages, there is still an extra step before you can use the messages. You have to type in the Webshell, in the catkin_ws, the following command: source devel/setup.bash. 

This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.

If you don't do this, it might give you an import error, saying it doesn't find the message generated.

If your compilation goes fine, you should see something similar to this:



HINT 2: To verify that your message has been created successfully, type in your webshell rosmsg show Age. If the structure of the Age message appears, it will mean that your message has been created successfully and it's ready to be used in your ROS programs.
Execute in WebShell #1

rosmsg show Age
WebShell #1 Output

user ~ $ rosmsg show Age
[topic_ex/Age]:
float32 years
float32 months
float32 days
To use Custom Messages in Cpp files
You will have to add to your CMakeLists.txt the following extra lines to compile and link your executable ( in this example its called publish_age.cpp ) :

add_executable(publish_age src/publish_age.cpp)
add_dependencies(publish_age ${publish_age_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_age
   ${catkin_LIBRARIES}
 )
add_dependencies(publish_age topic_ex_generate_messages_cpp)
As you can see, there is nothing new here except for the last line:

add_dependencies(publish_age topic_ex_generate_messages_cpp)
And why do you need to add this extra add_dependencies()? Well, this is to make sure that all the messages contained in the package (topic_ex) are compiled before we create the publish_age executable. In this case, the publish_age executable uses the custom message we have just created: Age.msg. So... what would happen if we try to build the executable before those messages are built? Well, it would fail, of course. Then, with this line, you will make sure that the messages are built before trying to build your executable.

Topics Quiz


With all you've learned during this course, you're now able to do a small Quiz to put everything together. Subscribers, Publisher, Messages... you will need to use all of this concepts in order to succeed!

For evaluating this Quiz, we will ask you to perform different tasks. For each task, very specific instructions will be provided: name of the package, names of the launch files and Cpp scripts, topic names to use, etc.

It is VERY IMPORTANT that you strictly follow these instructions, since they will allow our automated correction system to properly test your Quiz, and assign a score to it. If the names you use are different from the ones specified in the exam instructions, your exercise will be marked as FAILED, even though it works correctly.

In this Quiz, you will create a code to make the robot avoid the wall that is in front of him. To help you achieve this, let's divide the project down into smaller units:

Create a Publisher that writes into the /cmd_vel topic in order to move the robot.
Create a Subscriber that reads from the /kobuki/laser/scan topic. This is the topic where the laser publishes its data.
Depending on the readings you receive from the laser's topic, you'll have to change the data you're sending to the /cmd_vel topic, in order to avoid the wall. This means, use the values of the laser to decide.
HINT 1: The data that is published into the /kobuki/laser/scan topic has a large structure. For this project, you just have to pay attention to the 'ranges' array.
Execute in WebShell #1

rosmsg show sensor_msgs/LaserScan
WebShell #1 Output

user ~ $ rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges <-- Use only this one
float32[] intensities
HINT 2: The 'ranges' array has a lot of values. The ones that are in the middle of the array represent the distances that the laser is detecting right in front of him. This means that the values in the middle of the array will be the ones that detect the wall. So in order to avoid the wall, you just have to read these values.
HINT 3: The laser has a range of 30m. When you get readings of values around 30, it means that the laser isn't detecting anything. If you get a value that is under 30, this will mean that the laser is detecting some kind of obstacle in that direction (the wall).
HINT 4: The scope of the laser is about 180 degrees from right to left. This means that the values at the beginning and at the end of the 'ranges' array will be the ones related to the readings on the sides of the laser (left and right), while the values in the middle of the array will be the ones related to the front of the laser.
Specifications

The name of the package where you'll place all the code related to the Quiz will be topics_quiz.
The name of the launch file that will start your program will be topics_quiz.launch.
The name of the ROS node that will be launched by your program will be topics_quiz_node.
Quiz Correction


When you have finished the Quiz, you can correct it in order to get a Mark. For that, just click on the following button at the top of this Notebook. 


IMPORTANT



Quizes can only be done once. This means that, once you correct your Quiz, and get a score for it, you won't be able to do it again and improve your score. So, be sure enough when you decide to correct your Quiz!
Additional material to learn more
ROS Topics: http://wiki.ros.org/Topics

ROS Messages: http://wiki.ros.org/Messages

msg Files: http://wiki.ros.org/msg

Publisher and Subscriber 1: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

Publisher and Subscriber 2: http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber




ROS IN 5 DAYS
Unit 2: Topics


Estimated time to completion: 2.5 hours 

What will you learn with this unit?

What is a Subscriber and how to create one
How to create your own message


Part 2: Subscriber
You've learned that a topic is a channel where nodes can either write or read information. You've also seen that you can write into a topic using a publisher, so you may be thinking that there should also be some kind of similar tool to read information from a topic. And you're right! That's called a subscriber. A subscriber is a node that reads information from a topic. Let's execute the next code:

Example 2.3

Create a new package named topic_subscriber_pkg. When creating the package, add as dependencies roscpp and std_msgs.

Inside the src folder of the package, create a new file named simple_topic_subscriber.cpp. Inside this file, copy the contents of simple_topic_subscriber.cpp

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

END Example 2.3

C++ Program {2.2}: simple_topic_subscriber.cpp

#include <ros/ros.h>
#include <std_msgs/Int32.h>
​
void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback);
    
    ros::spin();
    
    return 0;
}
END C++ Program {2.2}: simple_topic_subscriber.cpp


What's up? Nothing happened again? Well, that's not actually true... Let's do some checks.

Go to your webshell and type the following:

Execute in WebShell #1

rostopic echo /counter
You should see an output like this:

WebShell #1 Output

user ~ $ rostopic echo /counter
WARNING: no messages received and simulated time is active.
Is /clock being published?
And what does this mean? This means that nobody is publishing into the /counter topic, so there's no information to be read. Let's then publish something into the topic and see what happens. For that, let's introduce a new command:

rostopic pub <topic_name> <message_type> <value>
This command will publish the message you specify with the value you specify, in the topic you specify.

Open another webshell (leave the one with the rostopic echo opened) and type the next command:

Execute in WebShell #2

rostopic pub /counter std_msgs/Int32 5
Now check the output of the console where you did the rostopic echo again. You should see something like this: .

WebShell #1 Output

user ~ $ rostopic echo /counter
WARNING: no messages received and simulated time is active.
Is /clock being published?
data:
5
---
This means that the value you published has been received by your subscriber program (which prints the value on the screen).

Now check the output of the shell where you executed your subscriber code. You should now see something like this:



Before explaining everything with more detail, let's explain the code you executed.

#include <ros/ros.h>
#include <std_msgs/Int32.h>
​
void counterCallback(const std_msgs::Int32::ConstPtr& msg) // Define a function called 'callback' that receives a                                                                // parameter named 'msg' 
{
  ROS_INFO("%d", msg->data); // Print the value 'data' inside the 'msg' parameter
}
​
int main(int argc, char** argv) {
​
    ros::init(argc, argv, "topic_subscriber"); // Initiate a Node called 'topic_subscriber'
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback); // Create a Subscriber object that will                                                                               // listen to the /counter topic and will
                                                                          // call the 'callback' function each time                                                                             // it reads something from the topic
    
    ros::spin(); // Create a loop that will keep the program in execution
    
    return 0;
}
So now, let's explain what has just happened. You've basically created a subscriber node that listens to the /counter topic, and each time it reads something, it calls a function that does a print of the msg. Initially, nothing happened since nobody was publishing into the /counter topic, but when you executed the rostopic pub command, you published a message into the /counter topic, so your subscriber has printed that number and you could also see that message in the rostopic echo output. Now everything makes sense, right?

Now let's do some exercises to put into practice what you've learned!

Exercise 2.4


Modify the previous code in order to print the odometry of the robot.
Data for Exercice 2.4


The odometry of the robot is published by the robot into the /odom topic.
You will need to figure out what message uses the /odom topic, and how the structure of this message is.
Remember to compile again your package in order to update your executable.
END Exercise 2.4

Solution Exercise 2.4

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit2 Topics Part2:Topics Part2 Solutions

END Solution Exercise 2.4

Exercise 2.5


Add to the {Exercice 2.4}, a C++ file that creates a publisher that indicates the age of the robot, to the previous package. 
For that, you'll need to create a new message called Age.msg. See the detailed description How to prepare CMakeLists.txt and package.xml for custom topic message compilation.
END Exercise 2.5

Solution Exercise 2.5

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions notebook for Unit2 Topics Part2:Topics Part2 Solutions

END Solution Exercise 2.5

How to Prepare CMakeLists.txt and package.xml for Custom Topic Message Compilation
Now you may be wondering... in case I need to publish some data that is not an Int32, which type of message should I use? You can use all ROS defined (rosmsg list) messages. But, in case none fit your needs, you can create a new one.

In order to create a new message, you will need to do the following steps:

Create a directory named 'msg' inside your package
Inside this directory, create a file named Name_of_your_message.msg (more information down)
Modify CMakeLists.txt file (more information down)
Modify package.xml file (more information down)
Compile
Use in code
For example, let's create a message that indicates age, with years, months, and days.

1) Create a directory msg in your package.

roscd <package_name>
mkdir msg
2) The Age.msg file must contain this:

float32 years
float32 months
float32 days
3) In CMakeLists.txt

You will have to edit four functions inside CMakeLists.txt:

find_package()
add_message_files()
generate_messages()
catkin_package()
I. find_package()
This is where all the packages required to COMPILE the messages of the topics, services, and actions go. In package.xml, you have to state them as build_depend.

HINT 1: If you open the CMakeLists.txt file in your IDE, you'll see that almost all of the file is commented. This includes some of the lines you will have to modify. Instead of copying and pasting the lines below, find the equivalents in the file and uncomment them, and then add the parts that are missing.
find_package(catkin REQUIRED COMPONENTS
       roscpp
       std_msgs
       message_generation   # Add message_generation here, after the other packages
)
II. add_message_files()
This function includes all of the messages of this package (in the msg folder) to be compiled. The file should look like this.

add_message_files(
      FILES
      Age.msg
    ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO
III. generate_messages()
Here is where the packages needed for the messages compilation are imported.

generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO
IV. catkin_package()
State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as exec_depend.

catkin_package(
      CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here
)
Summarizing, this is the minimum expression of what is needed for the CMakaelist.txt to work:

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.

cmake_minimum_required(VERSION 2.8.3)
project(topic_ex)
​
​
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
​
add_message_files(
  FILES
  Age.msg
)
​
generate_messages(
  DEPENDENCIES
  std_msgs
)
​
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)
​
include_directories(
  ${catkin_INCLUDE_DIRS}
)
4) Modify package.xml

Just add these 2 lines to the package.xml file.

<build_depend>message_generation</build_depend>
​
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
This is the minimum expression of the package.xml

Note: Keep in mind that the name of the package in the following example is topic_ex, so in your case, the name of the package may be different.

<?xml version="1.0"?>
<package format="2">
  <name>topic_ex</name>
  <version>0.0.0</version>
  <description>The topic_ex package</description>
​
​
  <maintainer email="user@todo.todo">user</maintainer>
​
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>message_generation</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>std_msgs</exec_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
​
  <export>
​
  </export>
</package>
5) Now you have to compile the msgs. To do this, you have to type in a WebShell:

Execute in WebShell #1

roscd; cd ..
catkin_make
source devel/setup.bash
VERY IMPORTANT: When you compile new messages, there is still an extra step before you can use the messages. You have to type in the Webshell, in the catkin_ws, the following command: source devel/setup.bash. 

This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.

If you don't do this, it might give you an import error, saying it doesn't find the message generated.

If your compilation goes fine, you should see something similar to this:



HINT 2: To verify that your message has been created successfully, type in your webshell rosmsg show Age. If the structure of the Age message appears, it will mean that your message has been created successfully and it's ready to be used in your ROS programs.
Execute in WebShell #1

rosmsg show Age
WebShell #1 Output

user ~ $ rosmsg show Age
[topic_ex/Age]:
float32 years
float32 months
float32 days
To use Custom Messages in Cpp files
You will have to add to your CMakeLists.txt the following extra lines to compile and link your executable ( in this example its called publish_age.cpp ) :

add_executable(publish_age src/publish_age.cpp)
add_dependencies(publish_age ${publish_age_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_age
   ${catkin_LIBRARIES}
 )
add_dependencies(publish_age topic_ex_generate_messages_cpp)
As you can see, there is nothing new here except for the last line:

add_dependencies(publish_age topic_ex_generate_messages_cpp)
And why do you need to add this extra add_dependencies()? Well, this is to make sure that all the messages contained in the package (topic_ex) are compiled before we create the publish_age executable. In this case, the publish_age executable uses the custom message we have just created: Age.msg. So... what would happen if we try to build the executable before those messages are built? Well, it would fail, of course. Then, with this line, you will make sure that the messages are built before trying to build your executable.

Topics Quiz


With all you've learned during this course, you're now able to do a small Quiz to put everything together. Subscribers, Publisher, Messages... you will need to use all of this concepts in order to succeed!

For evaluating this Quiz, we will ask you to perform different tasks. For each task, very specific instructions will be provided: name of the package, names of the launch files and Cpp scripts, topic names to use, etc.

It is VERY IMPORTANT that you strictly follow these instructions, since they will allow our automated correction system to properly test your Quiz, and assign a score to it. If the names you use are different from the ones specified in the exam instructions, your exercise will be marked as FAILED, even though it works correctly.

In this Quiz, you will create a code to make the robot avoid the wall that is in front of him. To help you achieve this, let's divide the project down into smaller units:

Create a Publisher that writes into the /cmd_vel topic in order to move the robot.
Create a Subscriber that reads from the /kobuki/laser/scan topic. This is the topic where the laser publishes its data.
Depending on the readings you receive from the laser's topic, you'll have to change the data you're sending to the /cmd_vel topic, in order to avoid the wall. This means, use the values of the laser to decide.
HINT 1: The data that is published into the /kobuki/laser/scan topic has a large structure. For this project, you just have to pay attention to the 'ranges' array.
Execute in WebShell #1

rosmsg show sensor_msgs/LaserScan
WebShell #1 Output

user ~ $ rosmsg show sensor_msgs/LaserScan
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges <-- Use only this one
float32[] intensities
HINT 2: The 'ranges' array has a lot of values. The ones that are in the middle of the array represent the distances that the laser is detecting right in front of him. This means that the values in the middle of the array will be the ones that detect the wall. So in order to avoid the wall, you just have to read these values.
HINT 3: The laser has a range of 30m. When you get readings of values around 30, it means that the laser isn't detecting anything. If you get a value that is under 30, this will mean that the laser is detecting some kind of obstacle in that direction (the wall).
HINT 4: The scope of the laser is about 180 degrees from right to left. This means that the values at the beginning and at the end of the 'ranges' array will be the ones related to the readings on the sides of the laser (left and right), while the values in the middle of the array will be the ones related to the front of the laser.
Specifications

The name of the package where you'll place all the code related to the Quiz will be topics_quiz.
The name of the launch file that will start your program will be topics_quiz.launch.
The name of the ROS node that will be launched by your program will be topics_quiz_node.
Quiz Correction


When you have finished the Quiz, you can correct it in order to get a Mark. For that, just click on the following button at the top of this Notebook. 


IMPORTANT



Quizes can only be done once. This means that, once you correct your Quiz, and get a score for it, you won't be able to do it again and improve your score. So, be sure enough when you decide to correct your Quiz!
Additional material to learn more
ROS Topics: http://wiki.ros.org/Topics

ROS Messages: http://wiki.ros.org/Messages

msg Files: http://wiki.ros.org/msg

Publisher and Subscriber 1: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29

Publisher and Subscriber 2: http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber



ROS IN 5 DAYS
Unit 4: ROS Actions


Estimated time to completion: 3 hours 

What will you learn with this unit?

What is a ROS action
How to manage the actions of a robot
How to call an action server
Part 1
1) Did you understand the previous sections about topics and services?
2) Are they clear to you?
3) Did you have a good breakfast today?

If your answers to all of those questions were yes, then you are ready to learn about ROS actions. Otherwise, go back and do not come back until all of those answers are a big yes. You are going to need it...

Quadrotor simulation
Before starting with ROS actions learning, let's have some fun with the quadrotor simulation.
Make the quadrotor take off and control it with the keyboard.
How would you do that?

By issuing the following commands:

First, you need to take off.

Execute in WebShell #1

rostopic pub /drone/takeoff std_msgs/Empty "{}"
Hit "CTRL+C" to stop it and to be able to type more commands. In this case, the commands to move the drone with the keyboard.

Execute in WebShell #1

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
To land the drone, just publish into the /drone/land topic:

Execute in WebShell #1

rostopic pub /drone/land std_msgs/Empty "{}"
Code Explanation #1

"rosrun": ROS command that allows you to run a ROS program without having to create a launch file to launch it (it is a different way from what we've been doing here).

"teleop_twist_keyboard": Name of the package where the ROS program is. In this case, where the python executable is.

"teleop_twist_keyboard.py": Python executable that will be run. In this case, it's an executable that allows you to input movement commands through the keyboard. When executed, it displays the instructions to move the robot.

End Code Explanation #1


Fig.4.1 - Ardrone moved with teleop_twist_keyboard.py.
Exercise 4.1


Try to takeoff, move, and land the drone using the keyboard, as shown in the gif in {Fig:4.1}
END Exercise 4.1

What are actions?
Actions are like asynchronous calls to services
Actions are the same as services. When you call an action, you are calling a functionality that another node is providing. Just the same as with services. The difference is that when your node calls a service, it must wait until the service finishes. When your node calls an action, it doesn't necessarily have to wait for the action to complete.

Hence, an action is an asynchronous call to another node's functionality.

The node that provides the functionality has to contain an action server. The action server allows other nodes to call that action functionality.
The node that calls to the functionality has to contain an action client. The action client allows a node to connect to the action server of another node.

Fig.4.2 - Action Interface Diagram showing the under-the-hood communication system
Now let's see an action in action (I'm so funny!)

Exercise 4.2


Go to a shell and launch the ardrone action server with the following command:
Keep this program running for the rest of the tutorial, since it is the one that provides the action server you are going to use.
Execute in WebShell #1, Leave It Running

roslaunch ardrone_as action_server.launch
Questions:

How do you know the whole list of topics available?
Execute in WebShell #2

rostopic list
How do you know the whole list of services available?
Execute in WebShell #2

rosservice list
How do you know the whole list of action servers available?
Execute in WebShell #2

rosaction list





WRONG GUESS!!!

Fig.4.3 - Didn't Say The Magic Word




In order to find which actions are available on a robot, you must do a rostopic list.

Execute in WebShell #2

rostopic list
WebShell #2 Output

user ~ $ rostopic list
...
...
/ardrone_action_server/cancel
/ardrone_action_server/feedback
/ardrone_action_server/goal
/ardrone_action_server/result
/ardrone_action_server/status
...
...
END Exercise 4.2

When a robot provides an action, you will see that in the topics list. There are 5 topics with the same base name, and with the subtopics cancel, feedback, goal, result, and status.

For example, in the previous rostopic list, the following topics were listed:

/ardrone_action_server/cancel
/ardrone_action_server/feedback
/ardrone_action_server/goal
/ardrone_action_server/result
/ardrone_action_server/status
This is because you previously launched the ardrone_action_server with the command roslaunch ardrone_as action_server.launch ( Excercice 4.2 ).

Every action server creates those 5 topics, so you can always tell that an action server is there because you identified those 5 topics.

Therefore, in the example above: 

ardrone_action_server: Is the name of the Action Server. 

cancel, feedback, goal, result and status: Are the messages used to comunicate with the Action Server.
Calling an action server
The ardrone_action_server action server is an action that you can call. If you call it, it will start taking pictures with the front camera, one picture every second, for the amount of seconds specified in the calling message (it is a parameter that you specify in the call).

Calling an action server means sending a message to it. In the same way as with topics and services, it all works by passing messages around.

The message of a topic is composed of a single part: the information the topic provides.
The message of a service has two parts: the goal and the response.
The message of an action server is divided into three parts: the goal, the result, and the feedback.
All of the action messages used are defined in the action directory of their package.

You can go to the ardrone_as package and see that it contains a directory called action. Inside that action directory, there is a file called Ardrone.action. That is the file that specifies the type of the message that the action uses.

Type in a shell the following commands to see the message structure:

Exercise 4.3


Type in a shell the following commands to see the message structure:
Execute in WebShell #2

roscd ardrone_as/action; cat Ardrone.action
WebShell #2 Output

user ~ $ roscd ardrone_as/action; cat Ardrone.action
#goal for the drone
int32 nseconds  # the number of seconds the drone will be taking pictures
---
#result
sensor_msgs/CompressedImage[] allPictures # an array containing all the pictures taken along the nseconds
---
#feedback
sensor_msgs/CompressedImage lastImage  # the last image taken
END Exercise 4.3

You can see in the previous step how the message is composed of three parts: 

goal: Consists of a variable called nseconds of type Int32. This Int32 type is a standard ROS message, therefore, it can be found in the std_msgs package. Because it's a standard package of ROS, it's not needed to indicate the package where the Int32 can be found. 
result: Consists of a variable called allPictures, an array of type CompressedImage[] found in the sensor_msgs package.
feedback: Consists of a variable called lastImage of type CompressedImage[] found in the sensor_msgs package.

You will learn in the second part of this chapter about how to create your own action messages. For now, you must only understand that every time you call an action, the message implied contains three parts, and that each part can contain more than one variable.

Actions provide feedback
Due to the fact that calling an action server does not interrupt your thread, action servers provide a message called the feedback. The feedback is a message that the action server generates every once in a while to indicate how the action is going (informing the caller of the status of the requested action). It is generated while the action is in progress.

How to call an action server
The way you call an action server is by implementing an action client.

The following is a self-explanatory example of how to implement an action client that calls the ardrone_action_server and makes it take pictures for 10 seconds.

Exercise 4.4

Create a new package named action_client_pkg. When creating the package, add as dependencies roscpp and actionlib.

Inside the src folder of the package, create a new file named ardrone_action_client.cpp. Inside this file, copy the contents of ardrone_action_client.cpp

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

Remember that you have to have the roslaunch ardrone_as action_server.launch running (probably in WebShell #1), otherwise this won't work because there will be NO action server to be connected to.
Exercise 4.4

C++ Program {4.4a}: ardrone_action_client.cpp

#include <ros/ros.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
​
int nImage = 0; //Initialization of a global variable
​
// Definition of the done calback. It is called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const ardrone_as::ArdroneResultConstPtr& result)
{
  ROS_INFO("The Action has been completed");
  ros::shutdown();
}
​
// Definition of the active callback. It is called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}
​
// Definition of the feedback callback. This will be called when feedback is received from the action server. It just // prints a message indicating a new message has been received
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
{
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_action_client"); // Initializes the action client node
  // Create the connection to the action server
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true);
  client.waitForServer(); // Waits until the action server is up and running 
​
  ardrone_as::ArdroneGoal goal; // Creates a goal to send to the action server
  goal.nseconds = 10; // Fills the goal. Indicates, take pictures along 10 seconds
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // sends the goal to the action server, specifying which                                                               // functions to call when the goal completes, when the                                                                 // goal becames active, and when feedback is received
  
  client.waitForResult();
  return 0;
}
Code Explanation C++ Program: {4.4a}


The code to call an action server is very simple:
First, you create a client connected to the action server you want:

actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true)
actionlib::SimpleActionClient<the_action_server_action_message> client("the_action_server_name", true)

Second parameter is the name of the action server you want to connecto to.
First parameter is the type of action message that it uses. The convention goes as follows: 

If your action message file was called Ardrone.action, then the type of action message you must specify is ArdroneAction. The same rule applies to any other type (R2Action, for an R2.action file or LukeAction for a Luke.action file). In our exercise it is: 

actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true)

Then you create a goal: 

ardrone_as::ArdroneGoal goal;

Again, the convention goes as follows: 

If your action message file was called Ardrone.action, then the type of goal message you must specify is ArdroneGoal. The same rule applies to any other type (R2Goal for an R2.action file or LukeGoal for a Luke.action file). 
Because the goal message requires to provide the number of seconds taking pictures, you must set that parameter in the goal class instance:

goal.nseconds = 10;

Next, you send the goal to the action server: 

client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

That sentence calls the action. In order to call it, you must specify 4 things:

The goal parameters
A done function to be called when the action finishes.
An active function to be called when the action starts.
A feedback function to be called from time to time to know the status of the action.
At this point, the action server has received the goal and started to execute it (taking pictures for 10 seconds). Also, feedback messages are being received. Every time a feedback message is received, the feedbackCb is executed. 

Finally, you wait for the result:


client.waitForResult();
So, at the end, you should get something like this in the webshell where you run the action client:



End Code Explanation {4.4a}

How to perform other tasks while the Action is in progress
You know how to call an action and wait for the result but... That's exactly what a service does! Then why are you learning actions?
Good point!
So the SimpleActionClient objects have two functions that can be used for knowing if the action that is being performed has finished, and how:

1) wait_for_result(): This function is very simple. When called, it will wait until the action has finished and returns a true value. As you can see, it's useless if you want to perform other tasks in parallel because the program will stop there until the action is finished.

2) get_state(): This function is much more interesting. When called, it returns an integer that indicates in which state is the action that the SimpleActionClient object is connected to.

0 ==> PENDING
1 ==> ACTIVE
2 ==> DONE
3 ==> WARN
4 ==> ERROR

This allows you to create a while loop that checks if the value returned by get_state() is 2 or higher. If it is not, it means that the action is still in progress, so you can keep doing other things.

Exercise 4.5

Inside the src folder of the package action_client_pkg, create 2 new files, named wait_for_result_test.cpp and no_wait_for_result_test.cpp. Inside this file, copy the contents of wait_for_result_test.cpp and no_wait_for_result_test.cpp.

Create 2 launch files for launching each one of the codes.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch files to run your executables and compare the results.

Remember that you have to have the roslaunch ardrone_as action_server.launch running (probably in WebShell #1), otherwise this won't work because there will be NO action server to be connected to.
END Exercise 4.5

C++ Program {4.5a}: wait_for_result_test.cpp

#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
​
int nImage = 0;
​
void doneCb(const actionlib::SimpleClientGoalState& state,
            const ardrone_as::ArdroneResultConstPtr& result)
{
  ROS_INFO("The Action has been completed");
  ros::shutdown();
}
​
// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}
​
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
{
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "wait_for_result");
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true);
  client.waitForServer();
​
  ardrone_as::ArdroneGoal goal;
  goal.nseconds = 10;
  
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  //client.waitForResult();
  
  ros::Rate loop_rate(2);
    
  while (client.waitForResult() != true)
  {
    ROS_INFO("Doing Stuff while waiting for the Server to give a result...");
    loop_rate.sleep();
  }
  
  return 0;
}
C++ Program {4.5b}: no_wait_for_result_test.cpp

#include <ros/ros.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
​
int nImage = 0;
​
void doneCb(const actionlib::SimpleClientGoalState& state,
            const ardrone_as::ArdroneResultConstPtr& result)
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The Action has been completed");
  ros::shutdown();
}
​
// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}
​
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
{
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_action_client");
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true);
  client.waitForServer();
​
  ardrone_as::ArdroneGoal goal;
  goal.nseconds = 10;
  
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  //client.waitForResult();
  
  ros::Rate loop_rate(2);
  actionlib::SimpleClientGoalState state_result = client.getState();
  ROS_INFO("[State Result]: %s", state_result.toString().c_str());
    
  while ( state_result == actionlib::SimpleClientGoalState::ACTIVE || state_result == actionlib::SimpleClientGoalState::PENDING )
  {
    ROS_INFO("Doing Stuff while waiting for the Server to give a result...");
    loop_rate.sleep();
    state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());
  }
    
  return 0;
}
Code Explanation C++ Programs: {4.5a} and {4.5b}

Essentially, the difference is that in the first program (4.5a), the log message ("Doing Stuff while waiting for the Server to give a result....") will never be printed, while in the 4.5b it will.

This is because in 4.5a, the program starts the while loop to check if the value returned by client.waitForResult() is True or False, but it will wait for a value that will only be returned when the Action has Finished. Therefore, it will never get inside the while loop because it will always return the value True.

On the other hand, in the 4.5b program, it checks if state_result == PENDING || state_result == ACTIVE. And because the function get_state() will return the current state of the action immediately, it allows other tasks to perform in parallel. In this case, printing the log message WHILE printing also the feedback printed by the Action.

At the end, you should get something like this:



End Code Explanation C++ Programs: {4.5a} and {4.5b}

You can get more information about ROS action clients in C++ in this client API

Exercise 4.6


Create a package that contains and launches the action client from Excercice {4.4a}: ardrone_action_client.cpp, from a launch file.
Add some code that makes the quadcopter move around while the action server has been called (in order to take pictures while the robot is moving).
Stop the movement of the quadcopter when the last picture has been taken (action server has finished).
Remember that you have to have the roslaunch ardrone_as action_server.launch running (probably in WebShell #1), otherwise this won't work because there will be NO action server to be connected to.
Data for Exercice 4.6


1) You can send Twist commands to the quadcopter in order to move it. These commands have to be published in /cmd_vel topic . Remember the TopicsUnit.
2) You must send movement commands while waiting until the result is received, creating a loop that sends commands at the same time that check for completion. In order to be able to send commands while the action is in progress, you need to use the SimpleActionClient function getState() .
END Exercise 4.6

Preempting a goal
It happens that you can cancel a goal previously sent to an action server prior to its completion.
Cancelling a goal while it is being executed is called preempting a goal

You may need to preempt a goal for many reasons, like, for example, the robot went mad about your goal and it is safer to stop it prior to the robot doing some harm. 

In order to preempt a goal, you send the cancel_goal to the server through the client connection.

client.cancelGoal()
Exercise 4.7

Inside the src folder of the package action_client_pkg, create a new files named cancel_goal_test.cpp. Inside this file, copy the contents of cancel_goal_test.cpp.

Create a launch files for launching the code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file in order to run the executable.

Remember that you have to have the roslaunch ardrone_as action_server.launch running (probably in WebShell #1), otherwise this won't work because there will be NO action server to be connected to.
END Exercise 4.7

C++ Program {4.6a}: cancel_goal_test.cpp

#include <ros/ros.h>
#include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
#include <actionlib/client/simple_action_client.h>
​
//int PENDING = 0;
//int ACTIVE = 1;
//int DONE = 2;
//int WARN = 3;
//int ERROR = 4;
​
int nImage = 0;
​
void doneCb(const actionlib::SimpleClientGoalState& state,
            const ardrone_as::ArdroneResultConstPtr& result)
{
  ROS_INFO("[State Result]: %s", state.toString().c_str());
  ROS_INFO("The Action has been completed");
  ros::shutdown();
}
​
// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}
​
void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
{
  ROS_INFO("[Feedback] image n.%d received", nImage);
  ++nImage;
}
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "cancel_goal_test");
  actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true);
  client.waitForServer();
​
  ardrone_as::ArdroneGoal goal;
  goal.nseconds = 10;
  
  client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
  //client.waitForResult();
  
  ros::Rate loop_rate(2);
  actionlib::SimpleClientGoalState state_result = client.getState();
  ROS_INFO("[State Result]: %s", state_result.toString().c_str());
  int counter = 0;
    
  while ( state_result == actionlib::SimpleClientGoalState::ACTIVE || state_result == actionlib::SimpleClientGoalState::PENDING )
  {
    ROS_INFO("Doing Stuff while waiting for the Server to give a result...");
    loop_rate.sleep();
    state_result = client.getState();
    ROS_INFO("[State Result]: %s", state_result.toString().c_str());
    ++counter;
    if (counter == 2)
    {
      ROS_INFO("Canceling goal...");
      client.cancelGoal();
      ROS_INFO("Goal canceled...");
      state_result = client.getState();
      ROS_INFO("[State Result]: %s", state_result.toString().c_str());
      ROS_INFO("counter = %d", counter);
    }
  }
    
  return 0;
}
Code Explanation C++ Program: {4.6a}

It's exactly the same code as the {4.5b}, except for the use of the cancelGoal() function.
This program counts to 2, and then it cancels the goal. This triggers the server to finish the goal and, therefore, the function getState() returns the value PREEMPTED.

You should see something like this in the webshell output of the client:



And this in the server webshell:



End Code Explanation C++ Program: {4.6a}

There is a known ROS issue with Actions. It issues a warning when the connection is severed. It normally happens when you cancel a goal or you just terminate a program with a client object in it. The warning is given in the Server Side.

[WARN] Inbound TCP/IP connection failed: connection from sender terminated before handshake header received. 0 bytes were received. Please check sender for additional details. 

Just don't panic, it has no effect on your program.

How does all that work?
You need to understand how the communication inside the actions works. It is not that you are going to use it for programming. As you have seen, programming an action client is very simple. However, it will happen that your code will have bugs and you will have to debug it. In order to do proper debugging, you need to understand how the communication between action servers and action clients works.

As you already know, an action message has three parts:

the goal
the result
the feedback

Each one corresponds to a topic and to a type of message.
For example, in the case of the ardrone_action_server, the topics involved are the following:

the goal topic: /ardrone_action_server/goal
the result topic: /ardrone_action_server/result
the feedback topic: /ardrone_action_server/feedback

Look again at the ActionClient+ActionServer communication diagram.


Fig.4.4 - Action Interface Diagram Copy
So, whenever an action server is called, the sequence of steps are as follows:

When an action client calls an action server from a node, what actually happens is that the action client sends to the action server the goal requested through the /ardrone_action_server/goal topic.

When the action server starts to execute the goal, it sends to the action client the feedback through the /ardrone_action_server/feedback topic.

Finally, when the action server has finished the goal, it sends to the action client the result through the /ardrone_action_server/result topic.

Now, let's do the following exercise in order to see how all this ballet happens underneath your programs.

Exercise 4.8


Read the whole exercise prior to executing it, because you have to get properly set up prior to the start of executing anything.
Execute in WebShell #1: Demonize action_server and echo the goal topic


(press [CTRL]+[C] to kill the ardrone_as action_server.launch if you had it still running)
roslaunch ardrone_as action_server.launch &
(to kill it when finished, use the command: rosnode kill /ardrone_as )

rostopic echo /ardrone_action_server/goal
Execute in WebShell #2: echo the feedback topic

rostopic echo /ardrone_action_server/feedback
Execute in WebShell #3: echo the result topic

rostopic echo /ardrone_action_server/result
Execute in WebShell #4: Launch the action server client

Execute the launch you created in Exercice 4.6, so that the drone starts to take pictures and move around.

Now do the following:

Quickly visit the terminal that contained the goal echo (WebShell #1). A message should have appeared indicating the goal you sent (with 10 seconds of taking pictures).
Quickly visit the feedback terminal containing the feedback echo (WebShell #2). A new message should be appearing every second. This is the feedback sent by the action server to the action client.
Quickly visit the result terminal (WebShell #3). If 10 seconds have not yet passed since you launched the action client, then there should be no message. If you wait there until the 10 seconds pass, you will see the result message sent by the action server to the action client appear.
Data for Excercice 4.8

Each one of those three topics have their own type of message. The type of the message is built automatically by ROS from the .action file.

For example, in the case of the ardrone_action_server the action file is called Ardrone.action.

When you compile the package (with catkin_make), ROS will generate the following types of messages from the Ardrone.action file:

ArdroneActionGoal
ArdroneActionFeedback
ArdroneActionResult
Each topic of the action server uses its associated type of message accordingly.

END Exercise 4.8

Exercise 4.9


Do a rostopic info of any of the action topics and check that the types really correspond with the ones indicated above.
END Exercise 4.9

Exercise 4.10


Due to the way actions work, you can actually call the ardrone_action_server action server publishing in the topics directly (emulating by hence, what the C++ code action client is doing). It is important that you understand this because you will need this knowledge to debug your programs.
Execute in WebShell #1: Run the action server


(press [CTRL]+[C] to kill the ardrone_as action_server.launch if you had it still running. If you had it demonized, use the command: rosnode kill /ardrone_as )
roslaunch ardrone_as action_server.launch
roslaunch ardrone_as action_server.launch
Let's activate the ardrone_action_server action server through topics with the following exercise. 

Use the webshell to send a goal to the /ardrone_action_server action server, and to observe what happens in the result and feedback topics. 

Execute in WebShell #2: Send goal to the action server

rostopic pub /[name_of_action_server]/goal /[type_of_the_message_used_by_the_topic] [TAB][TAB]
Expected Result for Exercice 4.10

You should see the same result as in exercice 4.8, with the difference that the goal has been sent by hand, publishing directly into the goal topic, instead of publishing through a C++ program.

Data for Excercice 4.10

You don't have to type the message by hand. Remember to use TAB-TAB to make ROS autocomplete your commands (or give you options).
Once you achieve that, and ROS autocompletes the message you must send, you will have to modify the parameter nseconds, because the default value is zero (remember that parameter indicates the number of seconds taking pictures). Move to the correct place of the message using the keyboard.
END Exercise 4.10

The axclient
Until now, you've learnt to send goals to an Action Server using these 2 methods:

Publishing directly into the /goal topic of the Action Server
Publishing the goal using C++ code

But, let me tell you that there's still one more method you can use in order to send goals to an Action Server, which is much easier and faster than the 2 methods you've learnt: using the axclient.

The axclient is, basically, a GUI tool provided by the actionlib package, that allows you to interact with an Action Server in a very easy and visual way. The command used to launch the axclient is the following:

rosrun actionlib axclient.py /<name_of_action_server>
Want do you think? Do you want to try it? Let's go then!

Exercise 4.11


Before starting with this exercise, make sure that you have your action server running. If it is not running, the axclient won't work.
Execute in WebShell #1: Run the action server


(press [CTRL]+[C] to kill the ardrone_as action_server.launch if you had it still running. If you had it demonized, use the command: rosnode kill /ardrone_as )
roslaunch ardrone_as action_server.launch
Now, let's launch the axclient in order to send goals to the Action Server. 

Execute in WebShell #2: Launch axclient

rosrun actionlib axclient.py /ardrone_action_server
In order to be able to visualize axclient, you will need to open the graphical interface window. To open this graphical interface window, just hit the icon with a screen in the IDE and a new tab will open. 



Now, you should see in your new tab something like this:


Now everything is settled, and you can start playing with axclient! For instance, you could send goals to the Action Server and visualize the different topics that take part in an Action, which you have learnt in this Chapter.

Expected Result for Exercice 4.11


Action in process:


Action succeeded:



Data for Exercice 4.11

Sometimes you may find that when you click the SEND GOAL button, or when you try to change the value of the goal you want to send, you can't interact with the axclient screen. If that's the case, just go to another tab and return again to the tab with axclient, and everything will work fine.
Exercise 4.11

Maybe you're a little bit angry at us now, because we didn't show you this tool before? Don't be!! 

The very simple reason why we didn't talk about this tool earlier is because we want you to learn how Actions really work inside. Once you have the knowledge, you are then ready to use the shortcuts.

Solutions

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions for the Actions Part 1:Actions Part 1 Solutions


ROS IN 5 DAYS
Unit 4: ROS Actions


Estimated time to completion: 2.5 hours 

What will you learn with this unit?

How to create an action server
How to build your own action message
Part 2
In the previous lesson, you learned how to call an action server creating an action client. In this lesson, you are going to learn how to create your own action server.


Fig.4.5 - Action Interface Diagram Copy 2
Writing an action server
Exercise 4.11: Test Fibonacci Action Server through Notebook

Create a new package named action_server_pkg. When creating the package, add as dependencies roscpp, actionlib and actionlib_msgs.

Inside the src folder of the package, create a new file named fibonacci_action_server.cpp. Inside this file, copy the contents of fibonacci_action_server.cpp.

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

The code below is an example of a ROS action server. When called, the action server will generate a Fibonacci sequence of a given order. The action server goal message must indicate the order of the sequence to be calculated, the feedback of the sequence as it is being computed, and the result of the final Fibonacci sequence.

END Exercise 4.11

C++ Program {4.11a}: fibonacci_action_server.cpp

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>
​
class FibonacciAction
{
protected:
​
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_; 
  std::string action_name_;
  // create messages that are used to publish feedback and result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
​
public:
​
  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }
​
  ~FibonacciAction(void)
  {
  }
​
  void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;
​
    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);
​
    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
​
    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
      // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
​
    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
​
​
};
​
​
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");
​
  FibonacciAction fibonacci("fibonacci");
  ros::spin();
​
  return 0;
}
Code Explanation C++ Program: {4.11a}

In this case, the action server is using an action message definition called Fibonacci.action. That message has been created by ROS into its actionlib_tutorials package.

protected:
​
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  actionlib::SimpleActionServer<actionlib_tutorials::FibonacciAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  actionlib_tutorials::FibonacciFeedback feedback_;
  actionlib_tutorials::FibonacciResult result_;
These are the protected variables of the action class. The node handle is constructed and passed into the action server during construction of the action. The action server is constructed in the constructor of the action and has been discussed below. The feedback and result messages are created for publishing in the action.

FibonacciAction(std::string name) :
  as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
  action_name_(name)
{
  as_.start();
}
In the class constructor, an action server is created. The action server takes arguments of a node handle, name of the action, and optionally an executeCB. In this example the action server is created with the arguments for the executeCB.

void executeCB(const actionlib_tutorials::FibonacciGoalConstPtr &goal)
{
Now the executeCB function referenced in the constructor is created. The callback function is passed a pointer to the goal message. Note: This is a boost shared pointer, given by appending "ConstPtr" to the end of the goal message type.

    ros::Rate r(1);
    bool success = true;
​
    // push_back the seeds for the fibonacci sequence
    feedback_.sequence.clear();
    feedback_.sequence.push_back(0);
    feedback_.sequence.push_back(1);
​
    // publish info to the console for the user
    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
Here the internals of the action are created. In this example ROS_INFO is being published to let the user know that the action is executing.

    // start executing the action
    for(int i=1; i<=goal->order; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
An important component of an action server is the ability to allow an action client to request that the current goal execution be cancelled. When a client requests that the current goal be preempted the action server should cancel the goal, perform necessary clean-up, and call the function setPreempted(), which signals that the action has been preempted by user request. Setting the rate at which the action server checks for preemption requests is left to the implementor of the server.

    // publish the feedback
      as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }
Here the Fibonacci sequence is put into the feedback variable and then published on the feedback channel provided by the action server. Then the action continues on looping and publishing feedback.

    if(success)
    {
      result_.sequence = feedback_.sequence;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
Once the action has finished computing the Fibonacci sequence the action notifies the action client that the action is complete by setting succeeded.

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");
​
  FibonacciAction fibonacci("fibonacci");
  ros::spin();
​
  return 0;
}
Finally the main function, creates the action and spins the node. The action will be running and waiting to receive goals.

End Code Explanation C++ Program: {4.11a}

Exercise 4.12a: Check Fibonacci action msg structure


Check the structure of the Fibonacci.action message definition by visiting the action directory of the actionlib_tutorials package.
END Exercise 4.12a

Exercise 4.12b: Watch feedback and result topic messages output from the action server


Launch again the C++ code above {4.11a} to have the Fibonacci server running. 

Then, execute the following commands in their corresponding WebShells.

Execute in WebShell #1: Echo the result topic

rostopic echo /fibonacci_as/result
Execute in WebShell #2: Echo the feedback topic

rostopic echo /fibonacci_as/feedback
Execute in WebShell #3: Manually send the goal to your Fibonacci server, publishing directly to the topic (as you learned in the previous chapter)

rostopic pub /fibonacci_as/goal actionlib_tutorials/FibonacciActionGoal [TAB][TAB]
Expected Result for Exercise 4.12b


After having called the action, the feedback topic should be publishing the feedback, and the result once the calculations are finished.


Data for Exercise 4.12b


You must be aware that the name of the messages (the class) used in the C++ code are called FibonacciGoal, FibonacciResult, and FibonacciFeedback, while the name of the messages used in the topics are called FibonacciActionGoal, FibonacciActionResult, and FibonacciActionFeedback. 
Do not worry about that, just bear it in mind and use it accordingly.
END Exercise 4.12b

Exercice 4.13: Create Package with Action Server that moves the Ardrone in the air, making a square


Create a package with an action server that makes the drone move in a square when called. 

Call the action server through the topics and observe the result and feedback. 

Base your code in the previous Fibonacci example {4.11a} and the client you did in Exercice 4.6 that moved the ardrone while taking pictures. 
Expected Result for Exercice 4.13


The result must show the ardrone doing a square in the air when the action server is called, as shown in the animation beneath {Fig:4.6} 
Data for Exercice 4.13

The size of the side of the square should be specified in the goal message as an integer.
The feedback should publish the current side (as a number) the robot is at while doing the square.
The result should publish the total number of seconds it took the drone to do the square
Use the Test.action message for that action server. Use the shell command find /opt/ros/kinetic/ -name Test.action to find where that message is defined. Then, analyze the different fields of the msg in order to learn how to use it in your action server.
END Exercice 4.13

Solution Exercise 4.13

Please Try to do it by yourself unless you get stuck or need some inspiration. You will learn much more if you fight for each exercise.



Follow this link to open the solutions for the Actions Part 2:Actions Part 2 Solutions

END Solution Exercise 4.13


Fig.4.6 - Ardrone moved through commands issed by an custom action server Ex 4.13
How to create your own action server message
It is always recommended that you use the action messages already provided by ROS. These can be found in the following ROS packages:

actionlib

Test.action
TestRequest.action
TwoInts.action
actionlib_tutorials

Fibonacci.action
Averaging.action


However, it may happen that you need to create your own type. Let's learn how to do it.

To create your own custom action message you have to:

1.- Create an action directory within your package.

2.- Create your Name.action message file.

The Name of the action message file will determine later the name of the classes to be used in the action server and/or action client. ROS convention indicates that the name has to be camel-case.

Remember the Name.action file has to contain three parts, each part separated by three hyphens.

#goal
package_where_message_is/message_type goal_var_name
---
#result
package_where_message_is/message_type result_var_name
---
#feedback
package_where_message_is/message_type feedback_var_name
If you do not need one part of the message (for example, you don't need to provide feedback), then you can leave that part empty. But you must always specify the hyphen separtors.
3.- Modify the file CMakeLists.txt and the package.xml to include action message compilation. Read the detailed description below.

How to prepare CMakeLists.txt and package.xml files for custom action messages compilation
You have to edit two files in the package, in the same way that we explained for topics and services:

CMakeLists.txt
package.xml
Modification of CMakeLists.txt
You will have to edit four functions inside CMakeLists.txt:

find_package()
add_action_files()
generate_messages()
catkin_package()
find_package()

All of the packages needed to COMPILE the messages of topic, services, and actions go here. 
In package.xml, you have to state them as built.

find_package(catkin REQUIRED COMPONENTS
      # your packages are listed here
      actionlib_msgs
      std_msgs # In case you use messages from std_msgs
)
add_action_files()

This function will contain all of the action messages from this package (which are stored in the action folder) that need to be compiled.
Place them beneath the FILES tag.

add_action_files(
      FILES
      Name.action
)
generate_messages()

The packages needed for the action messages compilation are imported here. Write the same here as you wrote in the find_package.

generate_messages(
      DEPENDENCIES
      actionlib_msgs
      std_msgs # In case you use messages from std_msgs
      # Your packages go here
)
catkin_package()

State here all of the packages that will be needed by someone that executes something from your package.
All of the packages stated here must be in the package.xml as exec_depend.

catkin_package(
      CATKIN_DEPENDS
      roscpp
      # Your package dependencies go here
)
Summarizing, You should end with a CMakeLists.txt similar to this:

cmake_minimum_required(VERSION 2.8.3)
project(my_custom_action_msg_pkg)
​
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs 
  actionlib_msgs
)
​
## Generate actions in the 'action' folder
add_action_files(
   FILES
   Name.action
 )
​
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs actionlib_msgs
 )
​
catkin_package(
 CATKIN_DEPENDS roscpp
)
​
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
Modification of package.xml:
1.- Add all of the packages needed to compile the messages.

If, for example, one of your variables in the .action file uses a message defined outside the std_msgs package, let's say nav_msgs/Odometry, you will need to import it. To do so, you would have to add as build_depend the nav_msgs package, adding the following line:

<build_depend>nav_msgs<build_depend>
2.- On the other hand, if you need a package for the execution of the programs inside your package, you will have to import those packages as exec_depend, adding the following lines:

<build_export_depend>nav_msgs<build_export_depend>
<exec_depend>nav_msgs<exec_depend>
When you compile custom action messages, it's mandatory to add the actionlib_msgs as build_dependency.

<build_depend>actionlib_msgs<build_depend>
When you use C++, it's mandatory to add the roscpp as exec_dependency.

<build_export_depend>roscpp<build_export_depend>
<exec_depend>roscpp<exec_depend>
This is due to the fact that the roscpp module is needed in order to run all of your C++ ROS code.

Summarizing, you should end with a package.xml file similar to this:

<?xml version="1.0"?>
<package format="2">
  <name>my_custom_action_msg_pkg</name>
  <version>0.0.0</version>
  <description>The my_custom_action_msg_pkg package</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib</build_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>actionlib</build_export_depend>
  <build_export_depend>actionlib_msgs</build_export_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>actionlib</exec_depend>
  <exec_depend>actionlib_msgs</exec_depend>
  <exec_depend>roscpp</exec_depend>
​
  <export>
  </export>
</package>
Finally, when everything is correctly set up, you just have to compile:

roscd; cd ..
catkin_make
source devel/setup.bash
rosmsg list | grep Name
You will get an output to the last command, similar to this:

my_custom_action_msg_pkg/NameAction
my_custom_action_msg_pkg/NameActionFeedback
my_custom_action_msg_pkg/NameActionGoal
my_custom_action_msg_pkg/NameActionResult
my_custom_action_msg_pkg/NameFeedback
my_custom_action_msg_pkg/NameGoal
my_custom_action_msg_pkg/NameResult
Note


Note that you haven't imported the std_msgs package anywhere. But you can use the messages declared there in your custom .actions. That's because this package forms part of the roscore file systems, so therefore, it's embedded in the compilation protocols, and no declaration of use is needed.
How to prepare CMakeLists.txt file for using custom action messages inside a CPP file
There is one extra step that you need to perform in order to use a custom action message. Its the same procedure as in custom topic messages or services.

Example Conditions:

You have a package called My_Examples_pkg, which has a file called Example1.cpp that you want to compile, that uses your recently created action Name.action.
This Name.action you defined it in the package my_custom_action_msg_pkg, that you have already compiled
Then your CMakeLists would need to llok something like this:

cmake_minimum_required(VERSION 2.8.3)
project(My_Examples_pkg)
​
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
)
​
## Generate actions in the 'action' folder
 add_action_files(
   FILES
   Name.action
 )
​
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
 )
​
catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp
)
​
###########
## Build ##
###########
​
​
add_executable(Example1 src/Example1.cpp)
add_dependencies(Example1 ${Example1_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(Example1
${catkin_LIBRARIES}
)
​
add_dependencies(Example1 my_custom_action_msg_pkg_generate_messages_cpp)
​
include_directories(
  ${catkin_INCLUDE_DIRS}
)
​
The line that allows the Example1.cpp find the NameAction.h generated while compiling the package my_custom_action_msg_pkg, is this line:

add_dependencies(Example1 my_custom_action_msg_pkg_generate_messages_cpp)
Actions Quiz


For evaluating this Quiz, we will ask you to perform different tasks. For each task, very specific instructions will be provided: name of the package, names of the launch files and C++ scripts, topic names to use, etc.

It is VERY IMPORTANT that you strictly follow these instructions, since they will allow our automated correction system to properly test your Quiz, and assign a score to it. If the names you use are different from the ones specified in the exam instructions, your exercise will be marked as FAILED, even though it works correctly.

Create a Package with an action server with custom action message to move ardone
The new action server will receive two words as a goal: TAKEOFF or LAND.
When the action server receives the TAKEOFF word, the drone will take off.
When the action server receives the LAND word, the drone will land.
As a feedback, it publishes once a second what action is taking place (taking off or landing).
When the action finishes, the result will return nothing.
Useful Data for the Quiz:

You need to create a new action message with the following structure:
string goal
---
---
string feedback
Specifications

The name of the package where you'll place all the code related to the Quiz will be actions_quiz.
The name of the launch file that will start your Action Server will be action_custom_msg.launch.
The name of the action will be /action_custom_msg_as.
The name of your Action message file will be CustomActionMsg.action.
Before correcting your Exam, make sure that all your Python scripts are executalbe. They need to have full execution permissions in order to be executed by our autocorrection system. You can give them full execution permissions with the following command:
chmod +x my_script.py
Quiz Correction


When you have finished the Quiz, you can correct it in order to get a Mark. For that, just click on the following button at the top of this Notebook. 


IMPORTANT



Quizes can only be done once. This means that, once you correct your Quiz, and get a score for it, you won't be able to do it again and improve your score. So, be sure enough when you decide to correct your Quiz!
Additional information to learn more
ROS Actions: http://wiki.ros.org/actionlib

How actions work: http://wiki.ros.org/actionlib/DetailedDescription


ROS IN 5 DAYS
Unit 5: Debugging Tools


Estimated time of completion: 1.5 hours 

What will you learn with this unit?

How can ROS What the F*ck help you debug
Add Debugging ROS logs
Filter ROS logs
Record and replay sensory data
Plot Topic Data
Draw connections between different nodes of your system
Basic use of RViz debugging tool
One of the most difficult, but important, parts of robotics is: knowing how to turn your ideas and knowledge into real projects. There is a constant in robotics projects: nothing works as in theory. Reality is much more complex and, therefore, you need tools to discover what is going on and find where the problem is. That's why debugging and visualization tools are essential in robotics, especially when working with complex data formats such as images, laser-scans, pointclouds or kinematic data. Examples are shown in {Fig-5.i} and {Fig-5.ii}.


Fig.5.i - Atlas Laser

Fig.5.ii - PR2 Laser and PointCloud
So here you will be presented with the most important tools for debugging your code and visualizing what is really happening in your robot system.

ROS What The F*ck!
It seems like a joke, but it isn't! Roswtf is a great tool to shed some light when you really don't know where to start solving a problem.

Go to the WebShell and type the following command:
Execute in WebShell #1

roswtf
WebShell #1 Output

user ~ $ roswtf
the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
No package or stack in context
================================================================================
Static checks summary:
​
Found 1 error(s).
​
ERROR ROS Dep database not initialized: Please initialize rosdep database with sudo rosdep init.
================================================================================
Beginning tests of your ROS graph. These may take awhile...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules
​
Online checks summary:
​
Found 2 warning(s).
Warnings are things that may be just fine, but are sometimes at fault
​
WARNING The following node subscriptions are unconnected:
 * /gazebo:
   * /gazebo/set_model_state
   * /gazebo/set_link_state
   * /iri_wam/iri_wam_controller/follow_joint_trajectory/cancel
   * /iri_wam/iri_wam_controller/follow_joint_trajectory/goal
   * /iri_wam/e_stop
   * /iri_wam/iri_wam_controller/command
​
WARNING These nodes have died:
 * urdf_spawner-4
In this particular case, it tells you {webshell-out-5.1} that the package rosdep hasn't been initialised, so you may have issues installing new ROS packages from the Internet. In this case, there is no problem because the system you are using (Robot Ignite Academy system) is not meant for installing anything. 

And this takes us to the question: What does roswtf do?

By default, it checks two ROS fields:

File-system issues: It checks enviromental variables, packages, and launch files, among other things. It looks for any inconsistencies that might be errors. You can use the command roswtf alone to get the system global status. But you can also use it to check particular launch files before using them.
Go to the WebShell and type the following command:
Execute in WebShell #1

roslaunch iri_wam_aff_demo false_start_demo.launch
WebShell #1 Output

user ~ $ roslaunch iri_wam_aff_demo false_start_demo.launch
logging to /home/user/.ros/log/fd58c97c-9068-11e6-9889-02c6d37ebbf9/roslaunch-ip-172-31-20-234-12087.log
Checking log directory for disk usage. This may take awhile.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.
​
started roslaunch server http://ip-172-31-20-234:38217/
​
SUMMARY
========
​
PARAMETERS
 * /rosdistro: indigo
 * /rosversion: 1.11.20
​
NODES
  /
    iri_wam_aff_demo (iri_wam_reproduce_trajectory/iri_wam_aff_demo_node)
    iri_wam_reproduce_trajectory (iri_wam_reproduce_trajectory/iri_wam_reproduce_trajectory_node)
​
ROS_MASTER_URI=http://localhost:11311
​
core service [/rosout] found
​
​
process[iri_wam_reproduce_trajectory-1]: started with pid [12111]
ERROR: cannot launch node of type [iri_wam_reproduce_trajectory/iri_wam_aff_demo_node]: can't locate node[iri_wam_aff_demo_node] in package [iri_wam_reproduce_trajectory]
Any clue? It just tells you that it can't locate your iri_wam_aff_demo_node. Now try roswtf on that launch file to get a bit more information on what might be the problem:

Go to the WebShell and type the following commands:
Execute in WebShell #1

roscd iri_wam_aff_demo/launch
roswtf false_start_demo.launch
WebShell #1 Output

user launch $ roswtf false_start_demo.launch
the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
[rospack] Error: the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
================================================================================
Static checks summary:
​
Found 2 error(s).
​
ERROR ROS Dep database not initialized: Please initialize rosdep database with sudo rosdep init.
ERROR Several nodes in your launch file could not be located. These are either typed incorrectly or need to be built:
 * node [iri_wam_aff_demo_node] in package [iri_wam_reproduce_trajectory]
To make roswtf yourlaunchfile.launch work, you need to go to the path where the file is. That's why you had to use the roscd command.
The error shown above it is telling you that roswtf can't find the iri_wam_aff_demo_node. It states that because it's a binary. It might be that you haven't compiled it yet or that you spelt something wrong. Essentially, it's telling you that there is no node iri_wam_aff_demo_node in the package iri_wam_reproduce_trajectory.

Online/graph issues: roswtf also checks for any inconsistencies in the connections between nodes, topics, actions, and so on. It warns you if something is not connected or it's connected where it shouldn't be. These warnings aren't necessarily errors. They are simply things that ROS finds odd. It's up to you to know if it's an error or if it's just the way your project is wired.
Go to the WebShell and type the following command:
Execute in WebShell #1

roswtf
WebShell #1 Output

user ~ $ roswtf
the rosdep view is empty: call 'sudo rosdep init' and 'rosdep update'
No package or stack in context
================================================================================
Static checks summary:
​
Found 1 error(s).
​
ERROR ROS Dep database not initialized: Please initialize rosdep database with sudo rosdep init.
================================================================================
Beginning tests of your ROS graph. These may take awhile...
analyzing graph...
... done analyzing graph
running graph rules...
... done running graph rules
​
Online checks summary:
​
Found 2 warning(s).
Warnings are things that may be just fine, but are sometimes at fault
​
WARNING The following node subscriptions are unconnected:
 * /gazebo:
   * /gazebo/set_model_state
   * /gazebo/set_link_state
   * /iri_wam/iri_wam_controller/follow_joint_trajectory/cancel
   * /iri_wam/iri_wam_controller/follow_joint_trajectory/goal
   * /iri_wam/e_stop
   * /iri_wam/iri_wam_controller/command
​
WARNING These nodes have died:
 * urdf_spawner-4
You executed this command at the start, but you didn't pay attention to the lower part warnings. These warnings are Graph issues.

It states that some subcribers are not connected to the topics that they are meant to be connected to: This is quite normal as they might be nodes that only connect at the start or in certain moments. No error here.
The second warning states that a node has died: This also is quite normal as nodes, like in this case, that only run when they spawn objects, die after being used. But ROS is so kind that it lets you know, just in case it shouldn't work that way.
ROS Debugging Messages and Rqt-Console
Logs allow you to print them on the screen, but also to store them in the ROS framework, so you can classify, sort, filter, or else.

In logging systems, there are always levels of logging, as shown in {Fig-5.1}. In ROS logs case, there are five levels. Each level includes deeper levels. So, for example, if you use Error level, all the messages for Error and Fatal will be shown. If your level is Warning, then all the messages for levels Warning, Error and Fatal will be shown.


Fig.5.1 - LOG Levels
DEBUG ==> ROS_DEBUG("This is a Debug Log");
INFO ==> ROS_INFO("This is an Info Log");
WARNING ==> ROS_WARN("This is a Warning Log");
ERROR ==> ROS_ERROR("This is an Error Log");
FATAL ==> ROS_FATAL("This is a Fatal Log");
Run the following C++ code:
Exercise 5.1


Create a new package named logs_test. When creating the package, add as dependencies roscpp.

Inside the src folder of the package, create a new file named logger_example.cpp. Inside this file, copy the contents of logger_example.cpp

Create a launch file for launching this code.

Do the necessary modifications to your CMakeLists.txt file, and compile the package.

Execute the launch file to run your executable.

C++ Program {5.1}: logger_example.cpp


#include <ros/ros.h>
#include <ros/console.h>
#include <stdlib.h> 
​
int main(int argc, char** argv) {
    
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
      ros::console::notifyLoggerLevelsChanged();
    }
​
    ros::init(argc, argv, "log_demo");
    ros::NodeHandle nh;
    ros::Rate loop_rate(0.5); // We create a Rate object of 2Hz
    
    while (ros::ok()) // Endless loop until Ctrl + C
    {
        ROS_DEBUG("There is a missing droid");
        ROS_INFO("The Emperors Capuchino is done");
        ROS_WARN("Help me Obi-Wan Kenobi, you're my only hope");
        int exhaust_number = rand() % 100 + 1;
        int port_number = rand() % 100 + 1;
        ROS_ERROR("The thermal exhaust port %d, right below the main port %d", exhaust_number, port_number);
        ROS_FATAL("The DeathStar Is EXPLODING");
        loop_rate.sleep();
        ros::spinOnce();
    }
    
    return 0;
}
END C++ Program {5.1}: logger_example.cpp


The best place to read all of the logs issued by all of the ROS Systems is: /rosout

Go to the WebShell and type the following command:
Execute in WebShell #1

rostopic echo /rosout
You should see all of the ROS logs in the current nodes, running in the system.

END Exercise 5.1


Exercise 5.2

1- Change the LOG level in the previous code {logger_example.cpp} and see how the different messages are printed or not in /rosout topic, depending on the level selected.

2- Remember that you will need to recompile the package each time you make a modification in the code.

END Exercise 5.2

As you can see, with only one node publishing every 2 seconds, the amount of data is big. Now imagine ten nodes, publishing image data, laser data, using the actions, services, and publishing debug data of your DeepLearning node. It's really difficult to get the logging data that you want. 

That's where rqt_console comes to the rescue.

Type in the WebShell #1 the roslaunch command for launching the Exercice 5.1 launch.
Go to the graphical interface window (hit the icon with a screen in the IDE) 


And type in the WebShell #2 : rqt_console, to activate the GUI log printer.
Execute in WebShell #1

roslaunch logs_test logger_example.launch
Execute in WebShell #2

rqt_console
You will get a window similar to {Fig-5.2} in the browser tab that opend when clicking on the icon:



Fig.5.2 - Rqt Console
The rqt_console window is divided into three subpanels.

The first panel outputs the logs. It has data about the message, severity/level, the node generating that message, and other data. Is here where you will extract all your logs data.
The second one allows you to filter the messages issued on the first panel, excluding them based on criteria such as: node, severity level, or that it contains a certain word. To add a filter, just press the plus sign and select the desired one.
The third panel allows you to highlight certain messages, while showing the other ones.
You have to also know that clicking on the tiny white gear on the right top corner, you can change the number of messages shown. Try to keep it as low as possible to avoid performance impact in your system.

Filter the logs so that you only see the Warning and Fatal messages of the node from exercice 5.1
You should see something like {Fig-5.3}:


Fig.5.3 - Rqt Console Filter
Plot topic data and Rqt Plot
This is a very common need in any scientific discipline, but especially important in robotics. You need to know if your inclination is correct, your speed is the right one, the torque readings in an arm joint is above normal, or the laser is having anomalous readings. For all these types of data, you need a graphic tool that makes some sense of all the data you are receiving in a fast and real-time way. Here is where rqt_plot comes in handy.

To Continue you should have stopped the Exercice 5.1 node launched in the WebShell #1

Go to the WebShell and type the following command to start moving the robot arm:
Execute in WebShell #1

roslaunch iri_wam_aff_demo start_demo.launch
Go to another Webshell and type the following command to see the positions and the effort made by each joint of the robot arm:
Execute in WebShell #2

rostopic echo /joint_states -n1
As you can probably see, knowing what's happening in the robots joints with only arrays of numbers is quite daunting.

So let's use the rqt_plot command to plot the robot joints array of data.

Go to the graphical interface and type in a terminal the following command to open the rqt_plot GUI:

Remmember to hit [CTRL]+[C] to stop the rostopic echo

Execute in WebShell #2

rqt_plot
You will get a window similar to {Fig-5.4}:


Fig.5.4 - Rqt Plot
In the topic input located in the top-left corner of the window, you have to write the topic structure that leads to the data that you want to plot. Bear in mind that in order to be plotted, the topic has to publish a number. Once written, you press the PLUS SIGN to start ploting the Topic. 

In the case that we want to plot the robot joints, we need to plot the topic /joint_states, which has the following structure (that you can already get by extracting the topic message type with rostopic info , and afterwards using rosmsg show command from Unit 2):

std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort                                                                                                        
Then, to plot the velocity of the first joint of the robot, we would have to type /joint_states/velocity[0]. 

You can add as many plots as you want by pressing the "plus" button.

Exercise 5.3

Plot in rqt_plot the effort made by the four first joints of the robot while it moves.
END Exercise 5.3

Node Connections and Rqt graph
Is your node connected to the right place? Why are you not receiving data from a topic? These questions are quite normal as you might have experienced already with ROS systems. Rqt_graph can help you figure that out in an easier way. It displays a visual graph of the nodes running in ROS and their topic connections. It's important to point out that it seems to have problems with connections that aren't topics.

Go to the graphical interface and type in a terminal the following command to open the rqt_graph GUI:

Remember to have in the WebShell #1 the roslaunch iri_wam_aff_demo start_demo.launch

Execute in WebShell #2

rqt_graph
You will get something like the following image:


Fig.5.5 - Rqt-Graph Result
In the diagram {Fig-5.5}, you will be presented with all of the nodes currently running, connected by the topics they use to communicate with each other. There are two main elements that you need to know how to use:

The refresh button: Which you have to press any time you have changed the nodes that are running:


The filtering options: These are the three boxes just beside the refresh button. The first element lets you select between only nodes or topics. The second box allows you to filter by names of nodes.


Here is an example where you filter to just show the /gazebo and the /joint_states_relay {Fig-5.6}:


Fig-5.6 - Rqt-Graph Result Filtered /gazebo and /joint_states_relay
Exercise 5.4

Create a package that launches a simple Topic publisher and a subscriber, and filter the ros_graph output to show you only the two nodes and the topic you are interested in.
END Exercise 5.4

You should get something like this {Fig-5.7}:


Fig-5.7 - Rqt-Graph from Exercise 5.7
Record Experimental Data and Rosbags
One very common scenario in robotics is the following: 

You have a very expensive real robot, let's say R2-D2, and you take it to a very difficult place to get, let's say The Death Star. You execute your mission there and you go back to the base. Now, you want to reproduce the same conditions to improve R2-D2s algorithms to open doors. But you don't have the DeathStar nor R2-D2. How can you get the same exact sensory readings to make your test? Well, you record them, of course! And this is what rosbag does with all the ROS topics generated. It records all of the data passed through the ROS topics system and allows you to replay it any time through a simple file.

The commands for playing with rosbag are:

To Record data from the topics you want: 

rosbag record -O name_bag_file.bag name_topic_to_record1 name_topic_to_record2 ... name_topic_to_recordN



To Extract general information about the recorded data: 

rosbag info name_bag_file.bag



To Replay the recorded data: 

rosbag play name_bag_file.bag


Replaying the data will make the rosbag publish the same topics with the same data, at the same time when the data was recorded.

Example 5.1

1- Go to the WebShell and type the following command to make the robot start moving if you don't haven't already:

Execute in WebShell #1

roslaunch iri_wam_aff_demo start_demo.launch
2- Go to another WebShell and go to the src directory. Type the following command to record the data from the /laser_scan:

Execute in WebShell #2

roscd; cd ../src
rosbag record -O laser.bag laser_scan
This last command will start the recording of data.

3- After 30 seconds or so, a lot of data have been recorded, press [CTRL]+[C] in the rosbag recording WebShell #2 to stop recording. Check that there has been a laser.bag file generated and it has the relevant information by typing:

Execute in WebShell #2

rosbag info laser.bag
4- Once checked, CTRL+C on the start_demo.launch WebShell #1 to stop the robot.

5- Once the robot has stopped, type the following command to replay the laser.bag (the -l option is to loop the rosbag infinitely until you CTRL+C):

Execute in WebShell #2

rosbag play -l laser.bag
6- Go to WebShell #3 and type the following command to read the ranges[100] of Topic /laser_scan :

Execute in WebShell #3

rostopic echo /laser_scan/ranges[100]
7- Type the following command in another WebShell , like WebShell #4. Then, go to the graphical interface and see how it plots the given data with rqt_plot:

Execute in WebShell #4

rqt_plot /laser_scan/ranges[100]
Is it working? Did you find something odd?
There are various things that are wrong, but it's important that you memorize this.

1- The first thing you notice is that when you echo the topic /laser_scan, you get sudden changes in values.

Try getting some more information on who is publishing in the /laser_scan topic by writing in the WebShell:

Remember to hit [CTRL]+[C] if there was something running there first.

Execute in WebShell #3

rostopic info /laser_scan
You will get something similar to this:

WebShell #3 Output

user ~ $ rostopic info /laser_scan
Type: sensor_msgs/LaserScan
​
Publishers:
 * /gazebo (http://ip-172-31-27-126:59384/)
 * /play_1476284237447256367 (http://ip-172-31-27-126:41011/)
​
Subscribers: None
As you can see, TWO nodes are publishing in the /laser_scan topic: gazebo (the simulation ) and play_x (the rosbag play).

This means that not only is rosbag publishing data, but also the simulated robot.

So the first thing to do is to PAUSE the simulation, so that gazebo stops publishing laser readings.

For that, you have to execute the following command:

Execute in WebShell #3

rosservice call /gazebo/pause_physics "{}"
And to UnPAUSE it again, just for you to know, just:

Execute in WebShell #3

rosservice call /gazebo/unpause_physics "{}"
With this, you should have stopped all publishing from the simulated robot part and only left the rosbag to publish.

Now you should be able to see a proper /laser_scan plot in the rqt_plot.

Still nothing?

2) Check the time you have in the rqt_plot. Do you see that, at a certain point, the time doesn't keep on going?

That's because you have stopped the simulation so that the time is not running anymore, apart from the tiny time frame in the rosbag that you are playing now.

Once rqt_plot reaches the maximum time, it stops. It doesn't return to the start, and therefore, if the values change outside the last time period shown, you won't see anything.

Take a look also at the rosbag play information of your currently running rosbag player

WebShell #2 Output

user ~ $ rosbag play -l laser.bag
[ INFO ] [1471001445.5575545086]: Opening laser.bag
​
Waiting 0.2 seconds after advertising topics... done.
​
Hit space to toggle paused, or 's' to stop.
[RUNNING] Bag Time: 61.676140 Duration: 41.099140 / 41.452000
In this example, you can see that the current time is 41.099 of 41.452 seconds recorded. And the rosbag time, therefore, will reach a maximum of around 62 seconds. 

62 seconds, in this case, is the maximum time the rqt_plot will show, and will start around 20 seconds. 

Therefore, you have to always CLEAR the plot area with the clear button in rqt_plot. 

By doing a CLEAR, you should get something similar to {Fig-5.8}:


Fig-5.8 - RosBag Rqt-Plot
To summarize: 

To use rosbag files, you have to make sure that the original data generator (real robot or simulation) is NOT publishing. Otherwise, you will get really weird data (the collision between the original and the recorded data). You have to also keep in mind that if you are reading from a rosbag, time is finite and cyclical, and therefore, you have to clear the plot area to view all of the time period.

rosbag is especially useful when you don't have anything in your system (neither real nor simulated robot), and you run a bare roscore. In that situation, you would record all of the topics of the system when you do have either a simulated or real robot with the following command:

rosbag record -a
This command will record ALL the topics that the robot is publishing. Then, you can replay it in a bare roscore system and you will get all of the topics as if you had the robot.

Before continuing any further, please check that you've done the following:

You have stopped the rosbag play by going to the WebShell #2 where it's executing and [CTRL]+[C]

You have unpaused the simulation to have it working as normal:

Execute in WebShell #3

rosservice call /gazebo/unpause_physics "{}"
END Example 5.1

Visualize Complex data and Rviz
And here you have it. The HollyMolly! The Milenium Falcon! The most important tool for ROS debugging....RVIZ. 

RVIZ is a tool that allows you to visualize Images, PointClouds, Lasers, Kinematic Transformations, RobotModels...The list is endless. You even can define your own markers. It's one of the reasons why ROS got such a great acceptance. Before RVIZ, it was really difficult to know what the Robot was perceiving. And that's the main concept: 

RVIZ is NOT a simulation. I repeat: It's NOT a simulation. 
RVIZ is a representation of what is being published in the topics, by the simulation or the real robot.

RVIZ is a really complex tool and it would take you a whole course just to master it. Here, you will get a glimpse of what it can give you.

Remember that you should have unpaused the simulations and stopped the rosbag as described in the rosbag section.

1- Type in WebShell #2 the following command:

Execute in WebShell #2

rosrun rviz rviz
2- Then go to the graphical interface to see the RVIZ GUI:

You will be greeted by a window like {Fig-5.9}:


Fig-5.9 - RVIZ Starting Window
Note: In case you don't see the lower part of Rviz (the Add button, etc.), double-click at the top of the window to maximize it. Then you'll see it properly.

You need only to be concerned about a few elements to start enjoying RVIZ.

Central Panel: Here is where all the magic happens. Here is where the data will be shown. It's a 3D space that you can rotate (LEFT-CLICK PRESSED), translate (CENTER MOUSE BUTTON PRESSED) and zoom in/out (LEFT-CLICK PRESSED).
LEFT Displays Panel: Here is where you manage/configure all the elements that you wish to visualize in the central panel. You only need to use two elements:
In Global Options, you have to select the Fixed Frame that suits you for the visualization of the data. It is the reference frame from which all the data will be referred to.
The Add button. Clicking here you get all of the types of elements that can be represented in RVIZ.
Go to RVIZ in the graphical interface and add a TF element. For that, click "Add" and select the element TF in the list of elements provided, as shown in {Fig-5.10}.


Fig-5.10 - RVIZ Add element
Go to the RVIZ Left panel, select as Fixed Frame the iri_wam_link_footprint and make sure that the TF element checkbox is checked. In a few moments, you should see all of the Robots Elements Axis represented in the CENTRAL Panel.

Now, go to a WebShell #1 and enter the command to move the robot:

Execute in WebShell #1

roslaunch iri_wam_aff_demo start_demo.launch
You should see something like this:


Fig-5.11 - RVIZ TF
In {Fig-5.11}, you are seeing all of the transformations elements of the IRI Wam Simulation in real-time. This allows you to see exactly what joint transformations are sent to the robot arm to check if it's working properly.

Now press "Add" and select RobotModel, as shown in {Fig-5.12}

Fig-5.12 - RVIZ Add Robot Model
You should see now the 3D model of the robot, as shown in {Fig-5.13}:


Fig-5.13 - RVIZ Robot Model + TF
Why can't you see the table? Or the bowl? Is there something wrong? Not at all! 

Remember: RVIZ is NOT a simulation, it represents what the TOPICS are publishing. In this case the models that are represented are the ones that the RobotStatePublisher node is publishing in some ROS topics. There is NO node publishing about the bowl or the table. 

Then how can you see the object around? Just like the robot does, through cameras, lasers, and other topic data. 

Remember: RVIZ shows what your robot is perceiving, nothing else.

Exercise 5.5

Add to RVIZ the visualization of the following elements:

What the RGB camera from the Kinect is seeing. TIP: The topic it has to read is /camera/rgb/image_raw. It might take a while to load the images, so just be patient.

What the Laser mounted at the end effector of the robot arm is registering. TIP: You can adjust the appearance of the laser points through the element in the LEFT PANEL.

What the PointCloud Camera / Kinect mounted in front of the robot arm is registering. TIP: You can adjust the appearance of the pointcloud points through the element in the LEFT PANEL. You should select points for better performance.

TIP: You should have a similar result as the one depicted beneath:

Notice that activating the pointcloud has a huge impact on the system performance. This is due to the huge quantity of data being represented. It's highly recommended to only use it with a high-end graphics card.

Play around with the type of representations of the laser, size, and so on, as well as with the pointcloud configuration.

END Exercise 5.5


Fig-5.14 - RVIZ Robot with camera and laser

Fig-5.15 - RVIZ Robot with camera, laser, and point-cloud
Congratulations! Now you are ready to debug any AstroMech out there!

Additional information to learn more
roswtf: http://wiki.ros.org/roswtf

Ros Logging System: http://wiki.ros.org/rospy/Overview/Logging

rqt_console: http://wiki.ros.org/rqt_console

rqt_plot: http://wiki.ros.org/rqt_plot

rqt_graph: http://wiki.ros.org/rqt_graph

Rosbag: http://wiki.ros.org/rosbag

Rviz: http://wiki.ros.org/rviz



ROS IN 5 DAYS
Course Project¶


Estimated time to completion: 10 hours 

What you will learn with this unit?

Practice everything you learn through the course
Put together everything you learn into a big project
Create a main program
Win the TurtleBot Race!
In this project, you will have to make a Turtlebot 2 Robot move along a maze faster than the other students. The fastest one will win a prize.

For this goal, you will have to apply all of the things that you are learning along the course. It's really important that you complete it because all of the structures that you create for this project will be asked about in our Official Exam. 

Get the robot out of the maze as fast as possible and you will get the prize. The ideal would be that Turtlebot goes out cleanly, but it may happen that you collide with the maze. You can use the collision detection to get data and help you in your strategy to get out.

Basically, in this project you will have to:

Apply all of the theory given in the course
Decide on a strategy to solve the problem
Implement this strategy in the simulation environment
Make as many tests as required in the simulation environment until it works
To achieve success in this project, we provide 5 steps below that you should follow with clear instructions and even solutions.

Also, remember to:

Create your packages and code in the simulation environment, as you have been doing throughout the course.
Use the consoles to gather information about the status of the simulated robot.
Use the IDE to create your programs and execute them through the consoles, observing the results on the simulation screen. You can use other consoles to watch calls to topics, services, or action servers.
Everything that you create in this unit will be automatically saved in your space. You can come back to this unit at any time and continue with your work from the point that you left it.
Every time you need to reset the position of the robot just press the restart button in the simulation window.
Use the debugging tools to try to find what is not working and why (for instance, the rviz tool is very useful for this purpose).
One final note: Because the program that you create should work with a real robot, if needed, you can't move the Turtlebot in a closed loop. This is because, in reality, the circuit could be different, the robot could not be as responsive, there might be errors in the readings, and so on. So in this project, you should create a program that can cope with all of these uncertainties.

What does Turtlebot Provide to Program It?
So the main question is, what can you do with Turtlebot from a ROS programming point of view? Which sensors and actuators does Turtlebot provide that will allow you to do the maze test?

Good question! Turtlebot provides the following sensors and actuators:

Sensors
Laser sensor: Turtlebot has a 2D Laser which provides information about the environment you are in. The value of the sensor is provided through the topic /kobuki/laser/scan.
Odometry: The odometry of the robot can be accessed through the /odom topic.
Actuators
Speed: You can send speed commands to move the robot through the /cmd_vel topic.
Now that you know the relevant topics of the robot, it is your job to figure out the types of messages and how to use them in order to make the robot do the strategy you want it to do.

Ideas to Start Working On
Here is a list of things that you can start from. You do not have to follow them. They are provided just in case you don't know how to start.

Start watching some of the messages that the sensor topics are publishing. Try to get an idea about the information they are providing. Move the robot in the simulation and see how those messages change their values. It is very important that you understand how changes in the robot produce changes in the topics.
Try to move the robot sending messages to the /cmd_vel (either through the console or through C++ programs).
Observe how the messages of the topics change when the robot moves around the environment, or when it crashes into a wall.
Is the odometry trustworthy? Can you move the robot the exact amount even when it collides with something?
Steps You Should Cover
These are the steps that you should follow throughout the duration of the project. These steps will ensure that you have practised and created all of the structures asked for in the final exam of this course. If you perform all of the steps mentioned here, you will find the exam feasible.

Step 1: Read and Write Topics (Dedicate 2 hours)
Step 2: Use topics through Services (Dedicate 3 hours)
Step 3: Use topics through Actions (Dedicate 4 hours)
Step 4: Create a main program to manage everything (Dedicate 1 hour)
NOTE: The 5th Step may not be required, but we have found some students that organize their code in such a way that they need this step. We have provided it here just in case you need it, but you don't have to use it.

Step 1: Read and Write Topics
This step has 3 actions for you to do:

Create a package called my_turtlebot_topics that will contain all of the programs related to topics
Create a topic publisher that allows you to move the Turtlebot.
Create two topic subscribers that extract the data that you need from the Odometry and the Laser.
So, let's get started.

1. Create package my_turtlebot_topics, with rospy as dependency.

2. Create the Topic Publisher to move Turtlebot.

To move Turtlebot, you need to publish in the topic /cmd_vel. 

It's important that you always encapsulate your topic subscribers and publishers inside classes. This will allow you to store values and manage the publishing callbacks easily.

First, you have to see if there is a topic like /cmd_vel running. 

Note: It will not always be this simple. In real robots, you will need to access the code to see what is the name of the topic that moves the robot, or even use rostopic info /name_of_topic to know which one it could be.

Execute in WebShell #1

rostopic list
As you may see, there is a /cmd_vel.
You, then, have to extract the type of message that /cmd_vel uses.

Execute in WebShell #1

rostopic info /cmd_vel
Test that it works by publishing different values:

Execute in WebShell #1

rostopic pub /cmd_vel message_type_of_cmd_vel [TAB][TAB]
Once you have the information, you are ready to create the class.
Create a C++ file in the src folder of the package you just created, "my_turtlebot_topics".
This file has to have not only the class, but also a way of testing that the class works.

3. Create Two Topic Subscribers that extract the data that you need from the Odometry and the Laser.

To get the Odometry data and the Laser data, you need to read the appropriate topics. Try finding them by yourself, first by typing:

Execute in WebShell #1

rostopic list
Have you found them? What type of message do they use? Are they the same as the ones listed here?
/kobuki/laser/scan, type = sensor_msgs/LaserScan 
/odom, type = nav_msgs/Odometry

Are they publishing? What's the data like?

Execute in WebShell #1

rostopic echo name_of_topic
Once you have the information, you are ready to create the classes for each one. 

Create two different C++ files in the src folder of the package that you just created, "my_turtlebot_topics".
These files have to have not only the class, but also a way of testing the class objects.

Remember that you need to move the Turtlebot to see the changes in both the /odom and the /kobuki/laser/scan topics. Use the previously created program to move it.

Step 2: Use Topics through Services
Now you need to take it a step further. Instead of having a topic subscriber/publisher on its own, you need to create a service that reads from the topics.
You have to do the following: 

Create a service that, when called, it tells you if the robot is about to hit an obstacle, through the laser data. It also has to return some extra information, such as the direction that the robot should move in next.

We divided this into 3 tasks:

Determine data
Modify the subscriber
Create the Server and Client
1. Determine data

Determine what data input you need ( request )
Determine what data you want the service to return ( response )
Then, you have to search for a previously done service message in the system. You can find them in the std_srvs or rospy_tutorials package. You can also find other service messages created in non-standard packages. Bear in mind that using packages that might not be installed in ROS systems or from third party packages is not recommened at this point. In this case, it's better to just generate your own service messages and use the std_msgs message types.
It's always preferable to use previously done messages, just because it's faster and you don't have to deal with compilation.

In this case, you need a service message of the following structure (DO NOT watch unless you are stuck):

# request, Empty because no data is needed
---
#response
bool movement_successfull
string extra_data
In this case, you have a service message that has this exact structure.
It's in the std_srvs package and is called Trigger.srv. This is no coincidence. This service structure is very useful because, normally, you ask a service to give you data without providing any input.

So you just need to create a service that uses this Trigger.srv that reads from the laser topic and tells you if you are about to crash or not. It will also tell you, based on the laser data, in which direction to move in now. 


2. Modify the /kobuki/laser/scan topic subscriber

Now you have to modify the /kobuki/laser/scan topic susbcriber to be able to tell you in what direction it is going to be the crash.

A way of evaluating the threshold that you consider it may be a potential crash, is by executing the code, and then controlling the Turtlebot with the: 

roslaunch turtlebot_teleop keyboard_teleop.launch

3. Create the Server and client that tell you if there is a potential crash, and in what direction to move

Why do you need to create a client, too? Well, this is not needed for your core program to run, but it's highly recommended because it allows you to test the server.

Step 3: Use Topics through Actions
Now you need to create an action that, when called, will start to save odometry data and check if the robot has exited the maze.
To accomplish that, you have to measure the distance from the starting point to the current position. If the distance is larger than the maze distance, you are out. A more elaborate one would be to consider the vector, and therefore, know if you exited correctly or you just jumped over a wall. 

The Action should also stop in case a certain period of time has passed without the robot exiting the maze. The task is then:

Create an action server that finishes when it has detected that the robot has exited the maze, or has been working for a certain period of time. Use only the /odom topic subscriber.

We divided it into 3 subtasks:

Define the action message
Create the action server, action client, and algorithm to exit the maze
Test it
1. The first thing you have to think about is what kind of message you need for this action to work as intended.

You need to call this action, without any input. 

It doesn't need to give feedback because the only thing that matters is that it returns the needed data to evaluate the distance. It needs to return the data used to calculate the distance for post-completion calculations. 

So the action message should look something like this (DO NOT watch this unless you are stuck):

#goal, empty                
---                             
#result, Odometry array             
nav_msgs/Odometry[] result_odom_array                
---                             
#feedback, empty
This message is, as you can see, custom. Therefore, you will need to compile the package.
The steps to do this are as follows:

Step 1: Create a new package called my_turtlebot_actions to store all the action servers and the message.
Step 2: Create an action directory, and within, an action message called record_odom.action.
Step 3: Make all of the needed changes to the package.xml and CMakeLists.txt files, in order to correctly compile the action message. These are the two files as they should be, if the only external dependency of your my_turtlebot_actions package is:

CMakeLists.txt

cmake_minimum_required(VERSION 2.8.3)
project(my_turtlebot_actions)
​
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Here go the packages needed to COMPILE the messages of topic, services and actions.
## in package.xml you have to state them as build
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  nav_msgs
)
​
## Generate actions in the 'action' folder
add_action_files(
   FILES
   record_odom.action
)
​
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   actionlib_msgs
   nav_msgs
 )
​
###################################
## catkin specific configuration ##
###################################
## Declare things to be passed to dependent projects
## State here all the packages that will be needed by someone that executes something from your package
## All the packages stated here must be in the package.xml as exec_depend
catkin_package(
  CATKIN_DEPENDS rospy nav_msgs
)
​
###########
## Build ##
###########
​
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
​
​
package.xml

<?xml version="1.0"?>
<package format="2">
  <name>my_turtlebot_actions</name>
  <version>0.0.0</version>
  <description>The my_turtlebot_actions package</description>
​
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO</license>
​
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>actionlib_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
​
  <build_export_depend>rospy</build_export_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  
  <exec_depend>rospy</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
​
  <export>
  </export>
</package>
Once you think you have it, execute the following commands in the WebShell:

Execute in WebShell #1

roscd;cd ..
catkin_make
source deve/setup.bash
rosmsg list | grep record_odom
Output in WebShell #1

my_turtlebot_actions/record_odomAction
my_turtlebot_actions/record_odomActionFeedback
my_turtlebot_actions/record_odomActionGoal
my_turtlebot_actions/record_odomActionResult
my_turtlebot_actions/record_odomFeedback
my_turtlebot_actions/record_odomGoal
my_turtlebot_actions/record_odomResult
rosmsg list | grep record_odom: This command lists all of the rosmsgs defined in the system and in your devel folder, and only filters with the grep command the ones with the name record_odom. All of the messages compiled from your packages are stored in the devel folder.
This is one of the best ways to know that your action message has been correctly compiled and is accessible for your entire ROS system.

2. Create the action server, the client, and the program to compute out of the maze.

This action server has to start recording the /odom topic and stop when a certain time period has passed, or the distance moved reaches a certain value.

Move your /odom topic subscriber to your my_turtlebot_actions package. This way, your server can use it easily.
Now create the action server and action client. It is the same here as in services: you don't need the client but it's very useful to test the server, and it also gives you a template for how to use it later in the core program.

Note: It may happen that, when you test it, you get the following error:

ImportError: No module named my_turtlebot_actions.msg





This error is quite common when you generate your own messages. It doesn't find the my_turtlebot_actions.msg. But you have compiled it and doing the rosmsg list returns the correct output. Then, why? Because in order to allow your program to find the messages, you have to compile them and execute the source devel/setup.bash. This script sets not only the ROS environment, but also other systems related to message generation.
So, in order to always make your messages work, do the following:

Execute in WebShell #1

catkin_make
source devel/setup.bash
Now you are ready to work on your action server.

3. Test it.

Launch it: 

rosrun my_turtlebot_actions rec_odom_action_server.py

Check that it's working: 

rosnode list | grep record_odom_action_server_node
rostopic list | grep rec_odom_as

Launch the client to test that it really works.

Leave it running until the time runs out. It should return in the client side all of the /odom recordings up until then.
Force the distance goal to be reached. Use the roslaunch turtlebot_teleop keyboard_teleop.launch. It should return the /odom topics recorded up until then as well.
Step 4: Create a Main Program to Manage Everything
So, finally, you have all of the tools needed to create a main program that does the following:

Calls a service that tells you if it is going to crash and in what direction you should move.
Moves the turtlebot based on the service response.
Checks if it has exited the maze or the time given has run out. If so, the program ends.
We have divided this into 3 sub-steps:

1. Create a package called my_turtlebot_main.

This package has to contain the main program and the C++ files that it needs. You might need to copy some files of other packages.

2. Create a launch file that launches the Action Server, the Service Server, and the main program.

Try it first with the node tags and, if it works, then generate launches for the action_server and the service_server and use the include tag.

3. Create the main program.

Use all of the data and knowledge that you have extracted from the clients of the server and action, to reuse as much code as possible.

Conclusion
Now try to optimise your system. Play with the rates and the detection strategy. Play with the movement and the "AI" used to decide what to do.
You especially have to know this project by heart because the exam will be very similar.



I'm finished, now what?
ROS Development Studio (ROSDS)


ROSDS is the The Construct web based tool to program ROS robots online. It requires no installation in your computer. Hence, you can use any type of computer to work on it (Windows, Linux or Mac). Additionally, a free account has already been created for you to try it out. So... what are you waiting for? You can access ROSDS here: http://rosds.online

You can use any of the many ROSjects available in order to apply all the things you've learned during the Course. You just need to paste the ROSject link to your browser's URL, and you will automatically have the simulation prepared in your ROSDS workspace.

Down below you can check some examples of the Public Rosjects we provide:

ARIAC Competition




ROSject Link: https://bit.ly/2t2px0t
Cartpole Reinforcement Learning




ROSject Link: https://bit.ly/2t2uGWr
RobotX Challenge




ROSject Link: https://bit.ly/2Tt4lw8
Want to learn more?


Once you have finished the course, you can still learn a lot of interesting ROS subjects.

Take more advanced courses that we offer at the Robot Ignite Academy, like Perception or Navigation. Access the Academy here: http://www.robotigniteacademy.com

Or, you can go to the ROS Wiki and check the official documentation of ROS, now with new eyes.

Thank You and hope to see you soon!






