Overview
++++++++


Functions provided
------------------

Various interfaces are provided to call various HSR functions from Python.
Shell functions that provide interactive operations are also available.


ROS Interface
++++++++++++++

Although this package communicates with the robot via ROS, it is designed so that the user does not need to be aware of them.

All environment variables (``ROS_MASTER_URI``, ``ROS_IP``, etc.) that affect ROS nodes at runtime have the same effect.


How to use
++++++++++

There are two ways to use it: as an interactive shell or as part of a Python program.

In interactive mode, by typing ``Ctrl-C`` (SIGINT),
It is possible to cancel a currently executed operation (e.g., autonomous movement or arm movement) in the middle of the operation.

In library mode, typing ``Ctrl-C`` will cause shutdown like a normal ROS node.

Interactive shell
-----------------

You can have the commands executed interactively.

.. sourcecode:: bash

    $ ihsrb

.. sourcecode:: python

    >>> HSR-B Interactive Shell 0.2.0
    >>>
    >>>       ____________  ______  _________       __  _______ ____
    >>>      /_  __/ __ \ \/ / __ \/_  __/   |     / / / / ___// __ \
    >>>       / / / / / /\  / / / / / / / /| |    / /_/ /\__ \/ /_/ /
    >>>      / / / /_/ / / / /_/ / / / / ___ |   / __  /___/ / _, _/
    >>>     /_/  \____/ /_/\____/ /_/ /_/  |_|  /_/ /_//____/_/ |_|
    >>>
    >>> In [1]: whole_body.move_to_go()
    >>>
    >>> In [2]:
    >>> Do you really want to exit ([y]/n)? y
    >>> Leaving HSR-B Interactive Shell

Use as a library
----------------

Import the ``hsrb_interface`` module if you want to use it as a library.

Below is a simple example of moving the arm joint.

.. sourcecode:: python

    import math
    import hsrb_interface

    with hsrb_interface.Robot() as robot:
        whole_body = robot.get('whole_body')
        goals = {
            'arm_lift_joint': 0.5,
            'arm_flex_joint': math.radians(-90)
        }
        whole_body.move_to_joint_positions(goals)

        whole_body.move_to_joint_positions(
            head_tilt_joint=math.radians(25)
        )

Communication Control
~~~~~~~~~~~~~~~~~~~~~

Establish communication with the robot by creating an instance of the ``hsrb_interface.Robot`` class.
Is the ``hsrb_interface.Robot.close()`` method called on all instances or,
If the instance is destroyed, communication is disconnected.

It also supports ``with`` statements to automate ``close()`` operations, as in the example above.

Action based tf use
~~~~~~~~~~~~~~~~~~~

By setting the use\_tf\_server keyword argument to True when instantiating the ``hsrb_interface.Robot`` class,
Action-based tf will be used.



Find out what features are available
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

By calling the ``hsrb_interface.Robot.list()`` method,
You can get a list of currently available resource items (mobile carts, cameras, joint control groups, etc.).

Acquisition of items
~~~~~~~~~~~~~~~~~~~~

By calling the ``hsrb_interface.Robot.get()`` method with the names given in the list above,
An object representing the corresponding item can be obtained. By setting attributes and calling methods on the acquired object,
you can use various functions of the robot.
You can use various functions of the robot by setting attributes and calling methods on the acquired object.

.. note:: See the Tutorial and API Reference for more details.

Related Tutorials
-----------------

See the relevant chapter in the HSR-B manual for tutorials and references.

tutorial
   7.Develop HSR
API Reference
   8.2.Python API