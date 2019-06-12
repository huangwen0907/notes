# ROBOT IGNITE

roslaunch is the command used to launch a ROS program. Its structure goes as follows:
> roslaunch <package_name> <launch_file>

As you can see, that command has two parameters: the first one is the name of the package that contains the launch file, and the second one is the name of the launch file itself (which is stored inside the package).

### Now... what's a package?
    ROS uses packages to organize its programs. You can think of a package as all the files that a specific ROS program contains; all its cpp files, python files, configuration files, compilation files, launch files, and parameters files.

    All those files in the package are organized with the following structure:
>    launch folder: Contains launch files:
            src folder: Source files (cpp, python)
            CMakeLists.txt: List of cmake rules for compilation
            package.xml: Package information and dependencies
    To go to any ROS package, ROS gives you a command named roscd. When typing: 
    >  roscd <package_name>  (It will take you to the path where the package package_name is located.)
    
### And... what's a launch file?
    Open the launch folder inside the turtlebot_teleop package and check the keyboard_teleop.launch file.
    
    >In the launch file, you have some extra tags for setting parameters and remaps. For now, don't worry about those tags and focus on the node tag.
    All launch files are contained within a <launch> tag. Inside that tag, you can see a <node> tag, where we specify the following parameters:
    
        > pkg="package_name" # Name of the package that contains the code of the ROS program to execute
        type="cpp_executable_name" # Name of the cpp executable file that we want to execute
        name="node_name" # Name of the ROS node that will launch our C++ file
        output="type_of_output" # Through which channel you will print the output of the program

### Create a package
Until now we've been checking the structure of an already-built package... but now, let's create one ourselves.

When we want to create packages, we need to work in a very specific ROS workspace, which is known as the catkin workspace. The catkin workspace is the directory in your hard disk where your own ROS packages must reside in order to be usable by ROS. Usually, the catkin workspace directory is called catkin_ws.

    > catkin_create_pkg <package_name> <package_dependecies>
The package_name is the name of the package you want to create, and the package_dependencies are the names of other ROS packages that your package depends on.

>In order to check that our package has been created successfully, we can use some ROS commands related to packages. For example, let's type:
~~~
       rospack list
        rospack list | grep my_package
        roscd my_package 
~~~
    rospack list: Gives you a list with all of the packages in your ROS system.
    rospack list | grep my_package: Filters, from all of the packages located in the ROS system, the package named my_package.
    roscd my_package: Takes you to the location in the Hard Drive of the package, named my_package. 


Sometimes (for example, in large projects) you will not want to compile all of your packages, but just the one(s) where you've made changes. You can do this with the following command:
>  catkin_make --only-pkg-with-deps <package_name>

This command will only compile the packages specified and its dependencies.

You can request information about a topic by doing:
> rostopic info <name_of_topic>.

A topic is a channel that acts as a pipe, where other ROS nodes can either publish or read information. Let's now see some commands related to topics (some of them you've already used).



Sometimes ROS won't detect a new package when you have just created it, so you won't be able to do a roslaunch. In this case, you can force ROS to do a refresh of its package list with the command:
> rospack profile

To get information about a message, you use the next command:
> rosmsg show <message>
    
    

### Modifying the CMakeLists.txt file
When coding with C++, it will be necessary to create binaries(executables) of your programs in order to be able to execute them. For that, you will need to modify the CMakeLists.txt file of your package, in order to indicate that you want to create an executable of your C++ file.
>add_executable(simple src/simple.cpp)
add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple
   ${catkin_LIBRARIES}
 )
 
 >add_executable(simple src/simple.cpp) 

    This line generates an executable from the simple.cpp file, which is in the src folder of your package. This executable will be placed by default into the package directory of your devel space, which is located by default at ~/catkin_ws/devel/lib/.
>add_dependencies(simple ${simple_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

    This line adds all the cmake target dependencies of the executable. It's used basically to allow CMake to correctly compile a package, making sure all the dependencies are in place.
    
> target_link_libraries(simple${catkin_LIBRARIES})

    This line specifies the libraries to use when linking a given target. For this case, it indicates to use the catkin libraries when linking to the executable you have created.
>rosnode list

    
## ROS Nodes
You've initiated a node in the previous code but... what's a node? ROS nodes are basically programs made in ROS. The ROS command to see what nodes are actually running in a computer is:
> rosnode list  

## Parameter Server
A Parameter Server is a dictionary that ROS uses to store parameters. These parameters can be used by nodes at runtime and are normally used for static data, such as configuration parameters.
>
 To get a list of these parameters, you can type:  > rosparam list 
 To get a value of a particular parameter, you can type: > rosparam get <parameter_name>  
 And to set a value to a parameter, you can type:   > rosparam set <parameter_name> <value >
    
# Roscore
In order to have all of this working, we need to have a roscore running. The roscore is the main process that manages all of the ROS system. You always need to have a roscore running in order to work with ROS. The command that launches a roscore is:

# Environment Variables
ROS uses a set of Linux system environment variables in order to work properly. You can check these variables by typing:
> export | grep ROS
### So now... what is ROS?
ROS is basically the framework that allows us to do all that we showed along this chapter. It provides the background to manage all these processes and communications between them... and much, much more!! In this tutorial you've just scratched the surface of ROS, the basic concepts. ROS is an extremely powerful tool. If you dive into our courses you'll learn much more about ROS and you'll find yourself able to do almost anything with your robots!

# Unit 2: Topics
What will you learn with this unit?
>What are ROS topics and how to manage them
What is a publisher and how to create one
What are topic messages and how they work

### Part 1: Publisher
    >Create a new package named topic_publisher_pkg. When creating the package, add as dependencies roscpp and std_msgs.
    Inside the src folder of the package, create a new file named simple_topic_publisher.cpp. Inside this file, copy the contents of simple_topic_publisher.cpp
    Create a launch file for launching this code.
    Do the necessary modifications to your CMakeLists.txt file, and compile the package.
    Execute the launch file to run your executable.

To get information about a certain topic, use the next command:
> rostopic info <topic_name>

To get information about a message, you use the next command:
> rosmsg show <message>

### Subscriber
    You've learned that a topic is a channel where nodes can either write or read information. You've also seen that you can write into a topic using a publisher, so you may be thinking that there should also be some kind of similar tool to read information from a topic. And you're right! That's called a subscriber. A subscriber is a node that reads information from a topic. Let's execute the next code:
>  rostopic pub <topic_name> <message_type> <value>

# How to Prepare CMakeLists.txt and package.xml for Custom Topic Message Compilation

>  In order to create a new message, you will need to do the following steps:

    1 Create a directory named 'msg' inside your package
    2 Inside this directory, create a file named Name_of_your_message.msg (more information down)
    3 Modify CMakeLists.txt file (more information down)
    4 Modify package.xml file (more information down)
    5 Compile
    6 Use in code
    
#### In CMakeLists.txt
You will have to edit four functions inside CMakeLists.txt:
>
    find_package()
    add_message_files()
    generate_messages()
    catkin_package()
    
I. find_package()
>This is where all the packages required to COMPILE the messages of the topics, services, and actions go. In package.xml, you have to state them as build_depend.
        find_package(catkin REQUIRED COMPONENTS
               roscpp
               std_msgs
               message_generation   # Add message_generation here, after the other packages
        )

II. add_message_files()
>This function includes all of the messages of this package (in the msg folder) to be compiled. The file should look like this.
    add_message_files(
          FILES
          Age.msg
        ) # Dont Forget to UNCOMENT the parenthesis and add_message_files TOO

III. generate_messages()
>Here is where the packages needed for the messages compilation are imported.
    generate_messages(
          DEPENDENCIES
          std_msgs
    ) # Dont Forget to uncoment here TOO
    
IV. catkin_package()
>State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as exec_depend.
    catkin_package(
          CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here
    )
    
## Modify package.xml
><build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>

    VERY IMPORTANT: When you compile new messages, there is still an extra step before you can use the messages. You have to type in the Webshell, in the catkin_ws, the following command: source devel/setup.bash. 
    This executes this bash file that sets, among other things, the newly generated messages created through the catkin_make.
    If you don't do this, it might give you an import error, saying it doesn't find the message generated.

## To use Custom Messages in Cpp files
You will have to add to your CMakeLists.txt the following extra lines to compile and link your executable ( in this example its called publish_age.cpp ) :
>add_executable(publish_age src/publish_age.cpp)
add_dependencies(publish_age ${publish_age_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_age
   ${catkin_LIBRARIES}
 )
add_dependencies(publish_age topic_ex_generate_messages_cpp)


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

void counterCallback(const std_msgs::Int32::ConstPtr& msg)
{
  ROS_INFO("%d", msg->data);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "topic_subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("counter", 1000, counterCallback);
    
    ros::spin();
    
    return 0;
}
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
rostopic pub <topic_name> <message_type> <value>
This command will publish the message you specify with the value you specify, in the topic you specify.

Open another webshell (leave the one with the rostopic echo opened) and type the next command:

Execute in WebShell #2

rostopic pub /counter std_msgs/Int32 5
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
generate_messages(
      DEPENDENCIES
      std_msgs
) # Dont Forget to uncoment here TOO
IV. catkin_package()
State here all of the packages that will be needed by someone that executes something from your package. All of the packages stated here must be in the package.xml as exec_depend.

catkin_package(
      CATKIN_DEPENDS roscpp std_msgs message_runtime   # This will NOT be the only thing here
)
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

<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
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
add_executable(publish_age src/publish_age.cpp)
add_dependencies(publish_age ${publish_age_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(publish_age
   ${catkin_LIBRARIES}
 )
add_dependencies(publish_age topic_ex_generate_messages_cpp)

    And why do you need to add this extra add_dependencies()? Well, this is to make sure that all the messages contained in the package (topic_ex) are compiled before we create the publish_age executable. In this case, the publish_age executable uses the custom message we have just created: Age.msg. So... what would happen if we try to build the executable before those messages are built? Well, it would fail, of course. Then, with this line, you will make sure that the messages are built before trying to build your executable.


# Services in ROS
why do you need to learn about services? 
Well, that's because for some cases, topics are insufficient or just too cumbersome to use. Of course, you can destroy the Death Star with a stick, but you will just spend ages doing it. Better tell Luke SkyWalker to do it for you, right? Well, it's the same with services. They just make life easier.

### Topics - Services - Actions
> To understand what services are and when to use them, you have to compare them with topics and actions.
Imagine you have your own personal BB-8 robot. It has a laser sensor, a face-recognition system, and a navigation system. The laser will use a Topic to publish all of the laser readings at 20hz. We use a topic because we need to have that information available all the time for other ROS systems, such as the navigation system. 

> The Face-recognition system will provide a Service. Your ROS program will call that service and WAIT until it gives you the name of the person BB-8 has in front of it.
The navigation system will provide an Action. Your ROS program will call the action to move the robot somewhere, and WHILE it's performing that task, your program will perform other tasks, such as complain about how tiring C-3PO is. And that action will give you Feedback (for example: distance left to the desired coordinates) along the process of moving to the coordinates.

> So... What's the difference between a Service and an Action?
Services are Synchronous. When your ROS program calls a service, your program can't continue until it receives a result from the service.
Actions are Asynchronous. It's like launching a new thread. When your ROS program calls an action, your program can perform other tasks while the action is being performed in another thread.

> Conclusion: Use services when your program can't continue until it receives the result from the service.


You can get more information about any service by issuing the following command:
> rosservice info /name_of_your_service
Here you have two relevant parts of data.
    rosservice info /execute_trajectory
    user ~ $ rosservice info /execute_trajectory
    Node: /iri_wam_reproduce_trajectory
    URI: rosrpc://ip-172-31-17-169:35175
    Type: iri_wam_reproduce_trajectory/ExecTraj
    Args: file

    Node: It states the node that provides (has created) that service.
    Type: It refers to the kind of message used by this service. It has the same structure as topics do. It's always made of package_where_the_service_message_is_defined / Name_of_the_File_where_Service_message_is_defined. In this case, the package is iri_wam_reproduce_trajectory, and the file where the Service Message is defined is called ExecTraj.
    Args: Here you can find the arguments that this service takes when called. In this case, it only takes a trajectory file path stored in the variable called file.


<launch>
  <include file="$(find iri_wam_reproduce_trajectory)/launch/start_service.launch"/>
  <node pkg ="iri_wam_aff_demo"
        type="iri_wam_aff_demo_node"
        name="iri_wam_aff_demo"
        output="screen">
  </node>
</launch>

1) The first part of the launch file calls another launch file called start_service.launch. 
2) The second part launches a node just as you learned in the ROS Basics Unit. That node is the one that will call the /execute_trajectory service in order to make the robot move.

### How to call a service
> rosservice call /the_service_name TAB-TAB

    Info: TAB-TAB means that you have to quickly press the TAB key twice. This will autocomplete the structure of the Service message to be sent for you. Then, you only have to fill in the values.


### How to know the structure of the service message used by the service?
>rosservice info /name_of_the_service

This will return you the name_of_the_package/Name_of_Service_message

Then, you can explore the structure of that service message with the following command:
>rossrv show name_of_the_package/Name_of_Service_message

Execute the following command to see what is the service message used by the /gazebo/delete_model service:
>rosservice info /gazebo/delete_model

# Service messages have TWO parts:

    REQUEST
    ---
    RESPONSE
    
    In the case of the DeleteModel service, REQUEST contains a string called model_name and RESPONSE is composed of a boolean named success, and a string named status_message. 

    The Number of elements on each part of the service message can vary depending on the service needs. You can even put none if you find that it is irrelevant for your service. The important part of the message is the three dashes ---, because they define the file as a Service Message
### Summarizing:
>
    The REQUEST is the part of the service message that defines HOW you will do a call to your service. This means, what variables you will have to pass to the Service Server so that it is able to complete its task.
    The RESPONSE is the part of the service message that defines HOW your service will respond after completing its functionality. If, for instance, it will return an string with a certaing message saying that everything went well, or if it will return nothing, etc...
    
    
# How to give a Service

# What are actions?

>    Actions are like asynchronous calls to services.Actions are the same as services. When you call an action, you are calling a functionality that another node is providing. Just the same as with services. The difference is that when your node calls a service, it must wait until the service finishes. When your node calls an action, it doesn't necessarily have to wait for the action to complete.
    Hence, an action is an asynchronous call to another node's functionality.
    The node that provides the functionality has to contain an action server. The action server allows other nodes to call that action functionality.The node that calls to the functionality has to contain an action client. The action client allows a node to connect to the action server of another node.
~~~
    #include <ros/ros.h>
    #include <ardrone_as/ArdroneAction.h> // Note: "Action" is appended
    #include <actionlib/client/simple_action_client.h>

    int nImage = 0; //Initialization of a global variable

    // Definition of the done calback. It is called once when the goal completes
    void doneCb(const actionlib::SimpleClientGoalState& state,
                const ardrone_as::ArdroneResultConstPtr& result)
    {
      ROS_INFO("The Action has been completed");
      ros::shutdown();
    }

    // Definition of the active callback. It is called once when the goal becomes active
    void activeCb()
    {
      ROS_INFO("Goal just went active");
    }

    // Definition of the feedback callback. This will be called when feedback is received from the action server. It just // prints a message indicating a new message has been received
    void feedbackCb(const ardrone_as::ArdroneFeedbackConstPtr& feedback)
    {
      ROS_INFO("[Feedback] image n.%d received", nImage);
      ++nImage;
    }

    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "drone_action_client"); // Initializes the action client node
      // Create the connection to the action server
      actionlib::SimpleActionClient<ardrone_as::ArdroneAction> client("ardrone_action_server", true);
      client.waitForServer(); // Waits until the action server is up and running 

      ardrone_as::ArdroneGoal goal; // Creates a goal to send to the action server
      goal.nseconds = 10; // Fills the goal. Indicates, take pictures along 10 seconds
      client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb); // sends the goal to the action server, specifying which                                                               // functions to call when the goal completes, when the                                                                 // goal becames active, and when feedback is received

      client.waitForResult();
      return 0;
    }
    
~~~    
   ###  Code Explanation C++ Program: {4.4a} 
~~~
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

~~~


## roswtf
    File-system issues: It checks enviromental variables, packages, and launch files, among other things. It looks for any inconsistencies that might be errors. You can use the command roswtf alone to get the system global status. But you can also use it to check particular launch files before using them.
    Online/graph issues: roswtf also checks for any inconsistencies in the connections between nodes, topics, actions, and so on. It warns you if something is not connected or it's connected where it shouldn't be. These warnings aren't necessarily errors. They are simply things that ROS finds odd. It's up to you to know if it's an error or if it's just the way your project is wired.
    

## ROS Debugging Messages and Rqt-Console
    Logs allow you to print them on the screen, but also to store them in the ROS framework, so you can classify, sort, filter, or else.
    In logging systems, there are always levels of logging, as shown in {Fig-5.1}. In ROS logs case, there are five levels. Each level includes deeper levels. So, for example, if you use Error level, all the messages for Error and Fatal will be shown. If your level is Warning, then all the messages for levels Warning, Error and Fatal will be shown.

#### rqt_console
    The rqt_console window is divided into three subpanels.
    The first panel outputs the logs. It has data about the message, severity/level, the node generating that message, and other data. Is here where you will extract all your logs data.
    The second one allows you to filter the messages issued on the first panel, excluding them based on criteria such as: node, severity level, or that it contains a certain word. To add a filter, just press the plus sign and select the desired one.
    The third panel allows you to highlight certain messages, while showing the other ones.
    You have to also know that clicking on the tiny white gear on the right top corner, you can change the number of messages shown. Try to keep it as low as possible to avoid performance impact in your system.

#### rqt_graph

#### Record Experimental Data and Rosbags
    One very common scenario in robotics is the following: 

    You have a very expensive real robot, let's say R2-D2, and you take it to a very difficult place to get, let's say The Death Star. You execute your mission there and you go back to the base. Now, you want to reproduce the same conditions to improve R2-D2s algorithms to open doors. But you don't have the DeathStar nor R2-D2. How can you get the same exact sensory readings to make your test? Well, you record them, of course! And this is what rosbag does with all the ROS topics generated. It records all of the data passed through the ROS topics system and allows you to replay it any time through a simple file.

    The commands for playing with rosbag are:
    To Record data from the topics you want: 

>    rosbag record -O name_bag_file.bag name_topic_to_record1 name_topic_to_record2 ... name_topic_to_recordN

    To Extract general information about the recorded data: 

>   rosbag info name_bag_file.bag

    To Replay the recorded data: 
>    rosbag play name_bag_file.bag
    
    Replaying the data will make the rosbag publish the same topics with the same data, at the same time when the data was recorded.

#### Visualize Complex data and Rviz
    RVIZ is a tool that allows you to visualize Images, PointClouds, Lasers, Kinematic Transformations, RobotModels...The list is endless. You even can define your own markers. It's one of the reasons why ROS got such a great acceptance. Before RVIZ, it was really difficult to know what the Robot was perceiving. And that's the main concept: 

    RVIZ is NOT a simulation. I repeat: It's NOT a simulation. 
    RVIZ is a representation of what is being published in the topics, by the simulation or the real robot.

    RVIZ is a really complex tool and it would take you a whole course just to master it. Here, you will get a glimpse of what it can give you.

    Remember that you should have unpaused the simulations and stopped the rosbag as described in the rosbag section.

[Additional information to learn more
roswtf: http://wiki.ros.org/roswtf
Ros Logging System: http://wiki.ros.org/rospy/Overview/Logging
rqt_console: http://wiki.ros.org/rqt_console
rqt_plot: http://wiki.ros.org/rqt_plot
rqt_graph: http://wiki.ros.org/rqt_graph
Rosbag: http://wiki.ros.org/rosbag
Rviz: http://wiki.ros.org/rviz]





# URDF


#### 1. Create a package
> catkin_create_pkg my_mira_description rospy rviz controller_manager gazebo_ros joint_state_publisher robot_state_publisher
#### 2. create the folders
>
    launch
    models
    rviz_config
    config
    urdf
    worlds
##### 3 create the .urdf file in the folder urdf
    In the example given, you have two links; in this case, two cylinders connected through a joint. Joints are what make elements of a robot turn and move. They are the articulations of the robot.
~~~
    Type: there are these types: revolute, continuous, prismatic, fixed, floating, and planar. You can learn more here: http://wiki.ros.org/urdf/XML/joint The joint selection will depend on how the physical model of your robot moves.
~~~
>
    Parent and Child: Here is where you set who is connected to your link.
    Origin: All of the coordinates and rpy are referenced to the Parent axis, not the child axis.
    Limit: This is a very important element, especially when you have to control a robot movement.
    Axis: Here you define around which Parent's AXIS the Child link will revolve. This, of course, depends on the type of joint; some of them don't have axis tags because they are irrelevant, such as the fixed joint.

<param name="robot_description" command="cat $(arg model)" />
    Here it is loading the URDF file to the param server variable called "robot_description." Bear in mind that if you are loading more than one robot, you will have to load them in different variables, like "robot1_description" and "robot2_description."
 
<!-- send fake joint values -->
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
<param name="use_gui" value="TRUE"/>
</node>
<!-- Combine joint values -->
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher"/>
    Start the jointstate publisher and the robotstate publisher. These will publish the TFs of the URDF of the robot links and joints.

<!-- Show in Rviz   -->
<!--<node name="rviz" pkg="rviz" type="rviz" args="-d $(find my_mira_description)/rviz_config/urdf.rviz"/>-->
<node name="rviz" pkg="rviz" type="rviz" args=""/>
    Run RVIZ. The part about loading your own RVIZ is commented. The first time you launch this, just save the RVIZ config file and then you will have all that is needed.

This command launches a totally empty RVIZ session, to which you will have to add the TF and RobotModel representations.
To visualize this, you will have to access the GraphicalTools by pressing the icon:
>roslaunch my_mira_description urdf_visualize.launch model:='$(find my_mira_description)/urdf/mira.urdf'


You will have to add two elements in RVIZ:

    RobotModel: In this case, just select the robot_description variable for the RobotDescription field.
    TF: It will turn green as soon as you select the correct fixed frame, in this case base_link.
    Save the RVIZ file so that you don't have to do this adding every time you launch.



## 5. Learn how to import your 3D CAD models to Gazebo
This is a very wide topic and there are many ways of doing it. Here you will learn just the basic elements needed for correctly importing 3D models.

When using a CAD tool, the final models have to be saved in STL format or Dae format. SolidWorks, for example, gives the option of STL. The only advantage with DAE is that DAE saves color information. But that's something that can be added afterwards in Blender.
Import the STL of the DAE models into Blender. Here you will have to set the origin of the model and the units. This is absolutely vital. The setting of the axis will determine how difficult it is to create the URDF model. As for the units, having a 10-foot high Mira Robot is not very realistic, apart from the effects on the inertias afterwards.
Once you have the axis and units set up, you can add a material to the blender model to add color.
Once done, you export it to dae format. Sometimes, blender doesn't save properly the first time, so you will have to import the new dae into blender again in an empty scene and then export it again to dae.
To know how to import a model from a CAD system to blender and set the origin, please have a look at our video tutorial on this matter:[https://youtu.be/aP4sDyrRzpU]
To correctly set the units in blender and other CAD techniques in Blender, this site is great: [http://www.rab3d.com/tut_blen_2-6x_608-1.php]
To add materials to blender, please refer to this tutorial: [https://youtu.be/rRdKj33Keec]

In this case, you will have already been provided with the models needed for this URDF:
>    roscd mira_description
    roscd mira_description
    cp ./models/mira/meshes/mira_body_v3.dae /home/user/catkin_ws/src/
    cp ./models/mira/meshes/mira_head_v5.dae /home/user/catkin_ws/src/
    cp ./models/mira/meshes/mira_eye_v4.dae /home/user/catkin_ws/src/
    roscd my_mira_description
    mkdir models;cd models;mkdir mira;cd mira;mkdir meshes
    cd /home/user/catkin_ws/src/;mv *.dae /home/user/catkin_ws/src/my_mira_description/models/mira/meshes/

    And now, place them inside the my_mira_description/models/mira/meshes/
    
Finally, you just have to replace the geometry that are now spheres, cylinders, and so on by the .dae files. Here is an example of how it would be done in the base_link case:

<link name="base_link">
    <visual>
        <origin rpy="0.0 0 0" xyz="0 0 0"/>
        <geometry>
            <mesh filename="package://my_mira_description/models/mira/meshes/mira_body_v3.dae"/>
        </geometry>
    </visual>
</link>

# Add Collisions:
One of the most crucial elements in being able to simulate a robot is how it interacts with the world around it.
At the moment, your URDF Mira Robot would be a Ghost in a simulation. There is a visual representation, but it can't interact with anything. It will go through the floor and the objects.

As you can see, the only difference is that there is a new tag called collision that specifies the collision geometry. This is the shape used for calculating the physical contacts.




# TF
### TF Publisher

~~~
def handle_turtle_pose(pose_msg, robot_name):
    // It creates an instance of the TransformBroadCaster
    br = tf.TransformBroadcaster()
    
// Publish the transform of the pose message:
// You have to publish each element of the position and orientation inside a parenthesis, otherwise it might not work.
// There is also a very important element, which is the rospy.Time.now(). This is because TF really depends on time to make //everything work and be able to play with past messages.Then, state the name of the child-frame you want to assign that model // (robot_name) and the parent-frame, which, in this case, is /world

    br.sendTransform((pose_msg.position.x,pose_msg.position.y,pose_msg.position.z),
                 (pose_msg.orientation.x,pose_msg.orientation.y,pose_msg.orientation.z,pose_msg.orientation.w),
                 rospy.Time.now(),
                 robot_name,
                 "/world")

代码：tf包提供了 TransformBroadcaster 的实现，以帮助使发布变换的任务更容易。
    要使用TransformBroadcaster，我们需要包含tf/transform_broadcaster.h头文件。

tf::Transform transform;
    作用：这里我们创建一个Transform对象，并将信息从2D乌龟姿势复制到3D变换中。
代码：transform.setRotation(q);
    作用：这里我们设置旋转。
代码：br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", turtle_name));
    作用：
    这是真正的工作完成的地方。
    使用TransformBroadcaster发送转换需要四个参数。
    首先，我们传递转换本身。
    现在我们需要给被发布的变换一个时间戳，我们只是用当前时间戳它，ros::Time::now().
    然后，我们需要传递我们创建的链接的父框架的名称，在这种情况下为“world”
    最后，我们需要传递我们正在创建的链接的子框架的名称，在这种情况下，这是乌龟本身的名称。
    注意：sendTransform和StampedTransform具有父对象和子对象的相反顺序。

~~~
## How to get the robot's position?
    This will depend on the localization system of your robot.
    It could be retrieved from the /odom topic publishing the odometry.
    It could be obtained from a navigation system.
    It could be calculated through GPS data.

## Subscriber
> listener = tf.TransformListener()
    (trans,rot) = listener.lookupTransform(follower_model_frame, model_to_be_followed_frame, rospy.Time(0))
可以过得两个坐标系之间转换的关系，包括旋转与平移。 



#### URDF
The URDF file is mainly divided into three parts: Links Definition, Joints Definition, and Materials/Colors Definition.

Here we have a Fixed joint that connects the torso_link to the head_pan_link.
<joint name="head_pan_servo" type="fixed">
    <parent link="torso_link"/>
    <child link="head_pan_link"/>
    <origin xyz="0 0 0.225" rpy="0 0 0"/>
</joint>

And here we have a Non-Fixed joint (continuous in this case). It connects the head_pan_link with the head_tilt_link.
<joint name="head_pan_joint" type="continuous">
    <parent link="head_pan_link"/>
    <child link="head_tilt_link"/>
    <origin xyz="0 0 0.045" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.7"/>
</joint>


# Static Transforms

>rosrun tf static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
    
    Where:
    x, y, and z are the offsets in meters
    yaw, pitch, and roll are the rotations in radians
    period_in_ms specifies how often to send the transform

You can also create a launch file that launches the command above, specifying the different values in the following way:
<launch>
    <node pkg="tf" type="static_transform_publisher" name="name_of_node" 
          args="x y z yaw pitch roll frame_id child_frame_id period_in_ms">
    </node>
</launch>




Step 1.1: Understanding the reference frames
The robot_localization node requires 4 different frames in order to work properly:

    base_link_frame: This is the frame that is in the robot itself, to which any sensor can be referenced. It is usually located in the center of the robot. It travels with it.
    odom_frame: This is the frame that is used to report the odometry.
    map_frame: This is the frame that is used to report a global position from a system that knows where the robot is. For instance, an AMCL system. If you are not using any external localization system, then this can be ignored.
    world_frame: This is the frame that references which one of the two previous frames is going to be used to get the absolute coordinates of the robot in the world.

Step 1.2: Adding the sensors to fuse
So, now it's time to indicate to the robot_localization node all the sensors we want to merge. The package accepts the following types of messages:

    nav_msgs/Odometry

    sensor_msgs/Imu

    geometry_msgs/PoseWithCovarianceStamped

    geometry_msgs/TwistWithCovarianceStamped

##### AMCL
    There exist several Localization systems in ROS, but the most known and widely used is definitely AMCL. AMCL is a probabilistic localization system for a robot moving in 2D. It implements the adaptive Monte Carlo Localization (AMCL) approach, which uses a particle filter to track the pose of a robot against a known map.

   s However, the purpose of this course is not to learn about AMCL, so we are not going to go into too much detail. If you want to learn more about it, you can go to the Navigation Course at our Robot Ignite Academy, here: https://www.robotigniteacademy.com/en/course/ros-navigation-in-5-days/details/

    For this chapter, we are going to focus on having an AMCL node working, so that we can combine it with the robot_localization node later. So, let's go for it!

#### Saving the map
    
    Another package available in the ROS Navigation Stack is the map_server package. This package provides the map_saver node, which allows us to access the map data from a ROS Service, and save it into a file.
You can save the built map at anytime by using the following command:
>
    rosrun map_server map_saver -f name_of_map
    rosrun map_server map_saver -f name_of_map
    
    You should end up with 2 new files: my_map.yaml and my_map.pgm.
    The PGM file is the one that contains the occupancy data of the map (the really important data), and the YAML file contains some metadata about the map, like the map dimensions and resolution, or the path to the PGM file.




































## launch文件的一般格式，参数：
<launch>
    <node .../>
    <rosparam ..../>
    <param .../>
    <include .../>
    <env .../>
    <remap .../>
    <arg.../>
</launch>


参数说明
<node >要启动的node参数
    pkg=''mypackage''
    type=''nodetype''
    name=''nodename''
    arg=''arg1 ....''(可选)
    respawn=''ture''(可选)如果节点停止，自动重启节点
    ns=''foo''（可选）在foo命名空间启动节点
    output=''log|screen''(可选)
<rosparam>操作yaml文件参数
    command=''load|dump|delete''(默认load)
    file=''$(find pkg-name)/path/foo.yaml''(load或dump命令)yaml文件的名字
    param=''param-name''参数名
<param>定义一个设置在参数服务器的参数，它可以添加到<node>中
    name=''namespace/name''
    value=''value''（可选）如果省略这个参数，则应指定一个文件(binfile/textfile)或命令
    type=''str|int|double|boot''(可选)指定参数的类型
    textfile=''$(find pkg-name)/path/file''(可选)   

    binfile=''$(find pkg-name)/path/file''()
    command=''(find pkg-name)/exe '$(find pkg-name)/arg.txt' ''(可选)exe是可执行文件（cpp、py），arg.txt是参数文件
<include>在当前launch文件中调用另一个launch文件
    file=''$(find pkg-name)/path/launch-file.launch''    
<env>设置节点的环境变量
    name=''environment-variable-name''
    value=''environment-variable-value''    
<remap>将一个参数名映射为另一个名字
    from=''original-name''
    to=''new-name''    
<arg>定义一个局部参数，该参数只能在一个launch文件中使用
    <arg name=''foo''/>声明一个参数foo，后面需要给它赋值
    <arg name=''foo'' default=''1''/>声明一个参数foo，如不赋值取默认值
    <arg name=''foo'' value=''bar''/>声明一常量foo，它的值不能修改
    
    
## 一．使用roslaunch打开world模型
roslaunch是一个启动ROS节点并插入机器人模型的标准方法．要建立一个空的world模型，我们只需要运行
> roslaunch gazebo_ros empty_world.launch

１．roslaunch参数
    你可以更改launch文件中的参数来改变Gazebo的启动行为：
>   paused：在暂停状态下启动Gazebo（默认为false）．
    use_sim_time：告诉ROS节点要求获取ROS话题/clock发布的时间信息（默认为true）．
    gui：启动Gazebo中的用户界面窗口（默认为true）．
    headless recording：启动Gazebo状态日志记录（默认为false）．
    debug：使用gdb以调试模式启动gzserver（默认为false）．
    verbose：用--verbose运行gzserver和gzclient，并将错误和警告打印到终端（默认为false）．
    
２．roslaunch命令示例通常情况下使用默认值就可以了，这里只是一个例子:
>  roslaunch gazebo_ros empty_world.launch paused:=true use_sim_time:=false gui:=true throttled:=false recording:=false debug:=true verbose:=true

3.运行其他demo worlds
   其他的demo worlds已经包含在gazebo_ros中了，它包括：
>  roslaunch gazebo_ros willowgarage_world.launch
    roslaunch gazebo_ros mud_world.launch
    roslaunch gazebo_ros shapes_world.launch
    roslaunch gazebo_ros rubble_world.launch


~`mud_world.launch的代码如下：`~
    <launch> 
        <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched --> 
        <include file="$(find gazebo_ros)/launch/empty_world.launch">   
            <arg name="world_name" value="worlds/mud.world"/>
            <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->  
            <arg name="paused" value="false"/>  
            <arg name="use_sim_time" value="true"/> 
            <arg name="gui" value="true"/>  
            <arg name="recording" value="false"/>  
            <arg name="debug" value="false"/> 
        </include>
    </launch>

~`继续查看mud_world.launch文件，我们现在看mud.world文件的内容．mud.world开头的一部分代码如下所示：`~
<sdf version="1.4">
    <world name="default">
      <include>
        <uri>model://sun</uri>
      </include>
      <include>
        <uri>model://ground_plane</uri>
      </include>
      <include>
        <uri>model://double_pendulum_with_base</uri>
        <name>pendulum_thick_mud</name>
        <pose>-2.0 0 0 0 0 0</pose>
      </include>
      ...
    </world>
  </sdf>
  
### ５．如何在你的计算机中找到world文件
world文件位于Gazebo资源路径的/worlds文件夹中．这个路径取决于你是如何安装Gazebo以及你的系统类型的．想要找到你的Gazebo资源路径，你可以运行如下命令：
> env | grep GAZEBO_RESOURCE_PATH

一个典型的路径是/usr/local/share/gazebo-1.9．在这个路径后面加上/worlds就可以了．

### 二．创建你自己的Gazebo ROS包
    在继续插入机器人之前，我们将首先讨论使在ROS下使用Gazebo的文件层次结构标准，以便我们可以做出以后的假设。现在，我们假设你的工作空间为catkin_ws．它的路径可能是：
> /home/user/catkin_ws/src

    关于机器人模型和描述的一切都在/MYROBOT_description包中，Gazebo中用到的所有的world文件和launch文件都在/MYROBOT_gazebo中．实际情况下用你机器人的名字的小写字母来代替'MYROBOT'．你的文件夹目录应该如下所示：
../catkin_ws/src
    /MYROBOT_description
        package.xml
        CMakeLists.txt
        /urdf
            MYROBOT.urdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /MYROBOT_gazebo
        /launch
            MYROBOT.launch
        /worlds
            MYROBOT.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
        
### ·创建一个自定义的world文件
>     你可根据自己的机器人和功能包创建自己的.world文件．这里我们将创建一个拥有地面，太阳和加油站的世界．下面是我们推荐的步骤．记得用你自己的机器人的名字代替MYROBOT ，如果你没有机器人的话可以直接用＇test＇代替：
    `１．创建一个ROS功能包名为MYROBOT_gazebo
    ２．在个包中，创建一个launch文件夹．
    ３．在launch文件夹中创建一个YOUROBOT.launch文件，写入如下代码：`

<launch>
  <!-- We resume the logic in empty_world.launch, changing only the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find MYROBOT_gazebo)/worlds/MYROBOT.world"/>
    <!-- more default parameters can be changed here -->
  </include>
</launch>

### ４．同样在这个功能包中，创建一个worlds文件夹，并创建一个MYROBOT.world 文件，写入如下代码：
<?xml version="1.0" ?>
<sdf version="1.4">
  <world name="default">
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://gas_station</uri>
      <name>gas_station</name>
      <pose>-2.0 7.0 0 0 0 0</pose>
    </include>
  </world>
</sdf>

### ５．通过如下命令你可以在Gazebo中启动你的自定义world文件
> . ~/catkin_ws/devel/setup.bash
roslaunch MYROBOT_gazebo MYROBOT.launch
### ·在Gazebo中编辑world文件
    你可以插入额外的模型到你的世界中并通过File->Save保存．结果会返回到你的ROS功能包中．

# 三．使用roslaunch插入URDF机器人
`有两种方法：`
一个是：通过ROS服务
>    第一种方法可以让您的机器人的ROS包在计算机和存储库检查之间更加便携。它允许你保持机器人相对于ROS包路径的位置，但也要求你写一个ROS服务程序．

另一个是：通过模型库
>   这种方法允许你的.world文件中包含你的机器人，这样看起来更整洁方便但是需要设定环境变量来将你的机器人添加到Gazebo的模型库．

我们会使用两种方法．但总的来说我们更推荐第一种方法．
`１．＂ROS服务＂`
    这个方法使用了一个叫做spawn_model的python程序来要求gazebo_ros节点向Gazebo中添加URDF．
    spawn_model 程序存储在gazebo_ros包中．你可以通过如下命令调用这个程序：
>  rosrun gazebo_ros spawn_model -file `rospack find MYROBOT_description`/urdf/MYROBOT.urdf -urdf -x 0 -y 0 -z 1 -model MYROBOT

`要看所有的spawn_model中的可变参数，运行：`
>   rosrun gazebo_ros spawn_model -h
１．１使用Baxter的URDF示例

    如果你没有URDF文件来测试，你可以从Rethink Robotics的baxter_common中下载baxter_description包．通过运行如下命令将这个包放在你的工作空间下：
> git clone https://github.com/RethinkRobotics/baxter_common.git
    
    现在你应该有一个叫做baxter.urdf的URDF文件在路径baxter_description/urdf/下，运行
>  rosrun gazebo_ros spawn_model -file `rospack find baxter_description`/urdf/baxter.urdf -urdf -z 1 -model baxter


接下来将它继集成到ROS启动文件中，打开MYROBOT_gazebo/launch/YOUROBOT.launch并在</launch>前添加：
<!-- Spawn a robot into Gazebo -->
     <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-file $(find baxter_description)/urdf/baxter.urdf -urdf -z 1 -model baxter" />
然后启动这个文件，你会得到一样的结果．
#### １．２使用PR2的XACRO示例
如果你的URDF不是XML形式而是XACRO形式．你可以对launch文件进行类似的修改．通过安装这个包你可以运行这个PR2示例．
对于ROS Jade版本：
> sudo apt-get install ros-jade-pr2-common

然后添加如下内容到先前的launch文件中：
<!-- Convert an xacro and put on parameter server -->
<param name="robot_description" command="$(find xacro)/xacro.py $(find pr2_description)/robots/pr2.urdf.xacro" />
<!-- Spawn a robot into Gazebo -->
<node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" args="-param robot_description -urdf -model pr2" />
运行这个文件得到如下结果：

