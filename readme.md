### Notes
- Run PyCharm only using the terminal and the command `pycharm-community` _(otherwise, rclpy will not be found since `~/.bashrc` isn't loaded and ROS2s `setup.bash` is not executed)_
- Use _`build.sh`_ to build the ROS2 package of the project
- Use _`run.sh`_ to run the code using ROS2 (project has to be built first)
- Use _`build_and_run.sh`_ to do both
- You do not have to build the project anew after every code change
- Running the code using PyCharms "quick run" works as long as the code isn't too complicated - if not, use _`run.sh`_