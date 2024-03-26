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

1. Run `git clone git@github.com:CDEI-Agro/miscellaneous.git` from yout terminal.

2. Make models directory in the gazebo folder: `mkdir -p ~/.gazebo/models`

3. Make the file `installation.sh` executable: `chmod +x path to file`.

4. Run the file: `bash -x installation.sh`

5. Check the terminal for any `yes` and `no` prompts while the script executes.

6. Only add the following lines to your bashrc file if you are on the platform's computer: 
    `export ROS_DOMAIN_ID=x`\
    where `x=1` for green moby and `x=2` for red moby

7. If you are on the computer that connects with moby platform add the following lines to your bashrc:
    ````
    set_moby_model() {
         case "$1" in
             "GREEN")
                 export ROS_DOMAIN_ID=1
            echo "MOBY model set to GREEN"
            ;;
        "RED")
            export ROS_DOMAIN_ID=2
            echo "MOBY model set to GREEN"
            ;;
        *)
            echo "Invalid robot color. Please specify either GREEN or RED."
            ;;
    esac
   }
      ````
   
    This creates a function called `set_moby_model` in your bashrc that 
    allows you to set the moby model you are communicating with.
    
    usage:
    
    ````
   set_moby_model GREEN or set_moby_model RED 

# Testing

1. Each time we need to run files from our package we need to source our ros2_ws:\
 `source /home/*your user name*/ros2_ws/install/setup.bash`. If we add it to the bashrc file
we won't need to run this command each time. Open bashrc `gedit ~/.bashrc` and add this line at the end: `source ros2_ws/install/setup.bash`.

2. Restart the terminal.

3. Plug the joysticks into any usb port.

4. Run `ros2 launch agri_bot sim_launch.launch.py use_sim_time:=true`.
  
5. Gazebo simulation will take some time to load for the first time. If it does't load re run the command after killing the previous one.

7. Once it starts you can use the joystick to  move the robot around.
