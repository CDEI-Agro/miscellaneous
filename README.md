# Instructions for setting up your Github account and SSH keys

1. Sign up on [Github](https://github.com/) using your UPC credentials and ask one of the team members to add you to the CDEI organization.
   
3. Create a a new ssh key for your account. In a terminal and run:

    `ssh-keygen -t ed25519 -C "your_email@example.com"` \
      replacing the email address with your email 
    
3. Save the key at the default location when prompted.
 
4. When asked to choose a password enter a password that you can memorize. Leaving it empty will not protect it with a password but setting a password/passphrase is recommended.

5. Copy the contents of your public key. Simnply run the following command in a terminal and copy the contents printed in the terminal:\
    `cat ~/.ssh/id_ed25519.pub`

 6. Add these contents to your github account [here](https://github.com/settings/keys). 
If the link doesn't open go to your githb account -> settings -> SSH nd GPG keys -> add new ssh key 
then paste the copied contents and name the key. Naming is important because you maybe usign multiple keys later down the line.
Alternatively, follow this [tutorial](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account) to
add a new key to your account. 

7. Once added, verify if the key has been added correctly by running the following command in a terminal:
    `ssh -T git@github.com`\
      When prompted, add github to known hosts by answering yes. If the key has been correctly added  you will see the following message printed:\
      `You've successfully authenticated`

# Instructions for setting up ROS and the Navigations stack:

1. Download this bash script [file](https://github.com/CDEI-Agro/miscellaneous/blob/main/getting_started.sh):

2. Make the file executable: `chmod +x path to file` .

3. Execute the file. Run this command in a terminal from folder where the file is located (usually downloads)L `./getting_started.sh`.

4. This should install ROS and all the necessary dependencies. Enter `Y` if and when prompted. Restart the terminal. 

5. Go to the src fodler of the newly created workspace: `cd ~/ros2_ws/src`

6. Clone agri_bot repo: `git clone git@github.com:CDEI-Agro/agri_bot.git`

7. Clone ouster driver  repo: `git clone -b ros2 --recurse-submodules git@github.com:CDEI-Agro/ouster-ros.git`

8. Go to the parent workspace folder: `cd ..`

9. Install all package dependencies using rosdep: `rosdep install --from-paths . --ignore-src -y`

10. If calling rosdep for the first time you might need to initalize rosdep. Follow the commands printed on the terminal.

11. Although we do not need turtlebot3 we may still be using some files from the package so install all turtlebot3 packages:
    `sudo apt install turtlebot3*`

12. Create a models folder in the gazebo folder to ad our mesh files to:
    `mkdir ~/.gazebo/models`

12. Build agri_bot packge. Call `colcon build --symlink-install --packages-select agri_bot`

13. Build ouster package: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`

# Testing

1. Each time we need to run files from our package we need to source our ros2_ws:\
 `source /home/*your user name*/ros2_ws/install/setup.bash`. If we add it to the bashrc file
we won't need to run this command each time. Open bashrc `gedit ~/.bashrc` and add this line at the end: `source ros2_ws/install/setup.bash`.

2. Restart the terminal.

3. Plug the joysticks into any usb port.

4. Run `ros2 launch agri_bot sim_launch.launch.py use_sim_time:=true`.
  
5. Gazebo simulation will take some time to load for the first time. If it does't load re run the command after killing the previous one.

7. Once it starts you can use the joystick to  move the robot around.
