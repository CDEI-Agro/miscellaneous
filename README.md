# Instructions for setting up your Github account and SSH keys
0. Install git if isn't already installed: `sudo apt install git`

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

# Instructions for setting up ROS and the Navigation stack:

1. Run `git clone git@github.com:CDEI-Agro/miscellaneous.git` from your terminal.

2. Make the file `installation.sh` executable: `chmod +x path to file`.

3. Run the file: `bash -x miscellaneous/installation.sh --platform` `x`

    replace x depending on the platform you are configuring:  
  
    `green` for MOBY green  
    `red` for MOBY red  
    `default` for your own computer or the computer that connects with these platforms

4. Check the terminal for any `yes` and `no` prompts while the script executes.

Runing this script will create a function called `set_moby_model` 
in your bashrc for any computer other than the platform. This allows 
you to set the moby model you are communicating with.

usage:

   set_moby_model GREEN or set_moby_model RED 

# Testing

1. Plug the joysticks into any usb port.

2. Run `ros2 launch agri_bot joy_control.launch.py sim:=true`.
  
3. Gazebo simulation will take some time to load for the first time. If it does't load re run the command after killing the previous one.

4. Once it starts you can use the joystick to  move the robot around.


# Issues:

If you encounter any key adding issues follow the instrutions [here](https://answers.ros.org/question/398460/how-to-add-a-pubkey/) to add apt key properly.

# Router setup instructions:

The sim card needs to be configured with the router to use internet:

1. Connect to the router wifi.
2. Enter the router via ssh: `ssh root@router_ip_address`.
3. Run the following command after turning on the correct sim by entering the router through the web app:

   `gsmctl -A AT+CPIN=\"PUK\",\"PIN\"`
