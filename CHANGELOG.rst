^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package boost_sml
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2020-10-28)
------------------
* Add system component to boost cmake include
* Add boost as dependency in package.xml
* Contributors: Tyler Weaver

0.1.0 (2020-10-13)
------------------
* [feature] Generate SML diagrams using boost.Graph (`#4 <https://github.com/PickNikRobotics/boost_sml/issues/4>`_)
* [feature] example source to enable catkin to build package that can be depended on (`#2 <https://github.com/PickNikRobotics/boost_sml/issues/2>`_)
* [fix] undefined reference error (`#6 <https://github.com/PickNikRobotics/boost_sml/issues/6>`_)
  * Change static constexpr into const to avoid compiler errors when building in debug mode doesn't inline variables.
* [fix] compilation errors when setting overriding CMAKE_CXX_STANDARD in catkin config. (`#5 <https://github.com/PickNikRobotics/boost_sml/issues/5>`_)
* [maint] move all headers into one directory (`#3 <https://github.com/PickNikRobotics/boost_sml/issues/3>`_)
* Contributors: JafarAbdi, Jere Liukkonen, Mark Moll, Tyler Weaver, picknik-jliukkonen
