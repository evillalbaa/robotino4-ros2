#!/bin/bash

##########################################################
# 		           FUNCTION DEFINITIONS                 	 #
##########################################################

# Function to install Visual Studio Code 
install_vscode() {
  echo "Visual Studio Code is not installed. Proceeding with installation..."
  sudo apt update 
  sudo snap install --classic code 
  echo "Visual Studio Code installation completed."
}

# Function to install terminator
install_terminator() {
  echo "Terminator is not installed. Proceeding with installation..."
  sudo apt update 
  sudo apt install -y terminator
  echo "Terminator installation completed."
}

# Function to install ROS 2 Humble
install_ros2() {
  echo "ROS 2 Humble is not installed. Proceeding with installation..."
  
  # Set locale
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  
  # Setup sources
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  
  # Intall ROS 2 packages
  sudo apt update 
  sudo apt upgrade -y
  sudo apt install ros-humble-desktop -y
  source /opt/ros/humble/setup.bash
  
  # Add ROS 2 repository
  sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
  
  echo "ROS 2 Humble installation completed."
}

# Function to install the required ROS 2 packages
install_ros_packages() {
  echo "Installing the required ROS 2 packages..."
  sudo apt update

  sudo apt install -y ros-humble-xacro
  sudo apt install -y ros-humble-gazebo-ros
  sudo apt install -y ros-humble-gazebo-ros-pkgs
  sudo apt install -y ros-humble-cartographer-ros
  sudo apt install -y ros-humble-nav2-map-server
  sudo apt install -y ros-humble-nav2-lifecycle-manager
  sudo apt install -y ros-humble-nav2-amcl
  sudo apt install -y ros-humble-nav2-controller
  sudo apt install -y ros-humble-nav2-planner
  sudo apt install -y ros-humble-nav2-behaviors
  sudo apt install -y ros-humble-nav2-bt-navigator
  sudo apt install -y ros-humble-nav2-navfn-planner
  sudo apt install -y ros-humble-dwb-core
  sudo apt install -y ros-humble-dwb-plugins
  sudo apt install -y ros-humble-dwb-critics

  echo "ROS 2 packages intallation completed."

}

# Function to install git 
install_git() {
  echo "Git is not installed. Proceeding with installation..."
  sudo apt update
  sudo apt install -y git
  echo "Git installation completed."
}

# Function to install colcon common extensions
install_colcon_extensions() {
  sudo apt update
  sudo apt install -y python3-colcon-common-extensions
  echo "colcon common extensions installation completed."
}

# Function to install python3 virtual enviroment library
install_python_virtual_env() {
  echo "Python 3 virtual enviroment library is not installed. Proceeding with installation..."
  sudo apt update
  sudo apt install -y python3.10-venv
  echo "python3 virtual enviroment library installation completed."
}

# Function to clone omni-robot-gui 
clone_omni_robot_gui_repo(){
  echo "Cloning the omni robot gui repository..."
  cd ~/
  git clone https://github.com/evillalbaa/omni-robot-gui.git
  echo "Omni robot GUI repository cloned successfully."
}

# Function to create ROS 2 workspace
create_ros2_workspace() {
  # Create ROS 2 workspace if it doesn't exist
  echo "Creating ROS 2 workspace..."
  mkdir -p ~/robotino4-ros2/src
  cd ~/robotino4-ros2
  echo "ROS 2 workspace created."
}

# Function to clone the repository into the ROS 2 workspace
clone_robotino_ros2_repository() {
  echo "Cloning the repository into the ROS 2 workspace..."
  cd ~/robotino4-ros2/src/
  git clone https://github.com/evillalbaa/robotino4-ros2.git
  echo "Repository cloned successfully."
}

# Function to move repository contents
move_repo_contents() {
  echo "Moving repository contents..."
  mv ~/robotino4-ros2/src/robotino4-ros2/* ~/robotino4-ros2/src/
  rm -rf ~/robotino4-ros2/src/robotino4-ros2
  echo "Repository contents moved and original folder removed."
}

# Function to move the gazebo models into the correspond folder
move_gazebo_models() {
  echo "Moving gazebo models..."
  SRC_DIR="~/robotino4-ros2/src/robotino_bringup/worlds/turtlebot3_world.world"
  DEST_DIR="~/.gazebo/models/turtlebot3_world.world"
  
  # Check if the model folder exist 
  if [ ! -e "$DEST_DIR" ]; then
      mkdir -p "$(dirname "$DEST_DIR")"
      cp -r "$SRC_DIR" "$DEST_DIR"
      
      echo "The file has been moved correctly"
  else
      echo "The file already exists, no action taken."
  fi
}

# Function to configurate the bash file
configure_bashrc() {
  echo "Configuring the bash file..."
  # Ensure ROS 2 Humble environment setup is sourced in .bashrc
  if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 Humble environment setup to .bashrc"
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  fi

  # Add colcon argcomplete to bashrc
  if ! grep -q "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" ~/.bashrc; then
    echo "Adding colcon argcomplete to .bashrc"
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
  fi

  # Add gazebo to bashrc
  if ! grep -q "source /usr/share/gazebo/setup.bash" ~/.bashrc; then
    echo "Adding gazebo to .bashrc"
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc
  fi

  # Add gazebo model path 
  if ! grep -q "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robotino4-ros2/src/" ~/.bashrc; then
    echo "Adding gazebo model path to .bashrc"
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robotino4-ros2/src/" >> ~/.bashrc
  fi
   
  # Add workspace setup to bashrc
  if ! grep -q "source ~/robotino4-ros2/install/setup.bash" ~/.bashrc; then
    echo "Adding ROS 2 workspace setup to .bashrc"
    echo "source ~/robotino4-ros2/install/setup.bash" >> ~/.bashrc
  fi
  
  # Comment any other workspace setup lines
  sed -i '/source ~\/.*\/install\/setup.bash/ { /source ~\/robotino4-ros2\/install\/setup.bash/!s/^/#/ }' ~/.bashrc

  # Source the updated .bashrc
  source ~/.bashrc
  
  echo "Bash file configuration finished"
  
}

# Function to build ROS 2 workspace
build_ros2_workspace() {
  echo "Building ROS 2 workspace..."
  cd ~/robotino4-ros2
  colcon build
  echo "ROS 2 workspace build completed."
  
  # Source the updated .bashrc
  source ~/.bashrc
}


##########################################################
# 			                  MAIN 			                  	 #
##########################################################

echo "Installing the necesary resources..."

# Check if Visual Studio Code is installed
if ! snap list | grep -q code; then
  install_vscode
else
  echo "Visual Studio Code is already installed."
fi

# Check if terminator is installed
if ! dpkg -l | grep -q terminator; then
  install_terminator
else
  echo "Terminator is already installed."
fi

# Check if ROS 2 Humble is installed
if ! dpkg -l | grep -q ros-humble; then
  install_ros2
else
  echo "ROS 2 Humble is already installed."
fi

# Installing the required ROS 2 packages
install_ros_packages

# Check if Git is install
if ! command -v git &> /dev/null; then
  install_git
else
  echo "Git is already installed."
fi

# Check if pyhton colcon extensions are installed
if ! dpkg -l | grep -q python3-colcon-common-extensions; then
   install_colcon_extensions
else
   echo "colcon common extensions are already installed."
fi

# Check if python3 virtual library is installed
# Comprobamos si python3-venv está instalado
if ! dpkg -l python3.10-venv &>/dev/null; then
  install_python_virtual_env
else
    echo "python3-venv is already installed"
fi

# Check if exit a ROS 2 workspace
if [ ! -d ~/robotino4-ros2 ]; then
   create_ros2_workspace
else
   echo "ROS 2 workspace already exists."
fi

# Clone robotino4-ros2 repository
clone_robotino_ros2_repository

# Move repository contents
move_repo_contents

# Move gazebo models
move_gazebo_models

# Bash file configuration
configure_bashrc

# Build ROS 2 workspace to ensure everything is set up
build_ros2_workspace

echo " "
echo "Installations and configurations are already completed!"
