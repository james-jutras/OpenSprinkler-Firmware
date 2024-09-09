#!/bin/bash

#####################################################################################################################################################
## https://github.com/OpenSprinkler/OpenSprinkler-Firmware
#####################################################################################################################################################
## build.sh  V2.2.1.100 2024-09-08
##
##    -c compile OpenSprinkler OPSI Version from source            	 	        Defaults to 'false'
##    -i install OpenSprinkler in /usr/local/bin/OpenSprinkler     	 	        Defaults to 'false'
##    -s create install OpenSprinkler Service 				                        Defaults to 'false'
##    -y install OpenSprinkler Service 				                                Defaults to 'false'
##    -r restart OpenSprinkler Service 	  	                                  Defaults to 'false'
##    -x skip the sudo apt update process  	  	                              Defaults to 'false'
##    -? help  =>display usagae                                
##
##  usage: ./build.sh -c -i
##
## NOTE: This script must be run as james
##
####################################################################################################################################################

usage="usage: $(basename "$0") [-c] [-i] [-s] [-y] [-r] [-x] [-?]
  -c compile_opensprinkler_flag           compile OpenSprinkler from source
  -i install_opensprinkler_flag           install OpenSprinkler in /usr/local/bin/OpenSprinkler
  -s create_service_flag                  create and install OpenSprinkler Service
  -y install_service_flag                 create and install OpenSprinkler Service
  -r restart_service_flag                 restart the OpenSprinkler Service 
  -x skip_apt_update_flag                 skip the sudo apt update process which can be time consuming   
  -? help                                 display usage       
  example: ./build.sh -c -i -s -r
"  

##############################################################################
# Function: ha_service_file_exists 'service_name'
#
# Parameter: service_name e.g. 'homeassistant' or 'OpenSprinkler.service'  
#
# Return: true (0) if the file exists, (1) otherwise
#
# usage: if ha_service_file_exists 'OpenSprinkler'; then ...
############################################################################## 
function service_file_exists
{
  if (( $# != 1 )) ; then
    echo "'usage: service_file_exists 'service_name'" >&2
    exit 1
  fi
    
  if [ -f "/etc/systemd/system/$1.service" ]; then
    return 0;     # 0 = true
  else
    return 1;     # 1 = false
  fi
}

##############################################################################
# Function: get_service_status 'service_name'
#
# Parameter1: get_service_status e.g. 'homeassistant' or 'OpenSprinkler.service'
#
# Return Values: 
# 0	  "program is running or service is OK"	            unit is active
# 1	  "program is dead and /var/run pid file exists"	  unit not failed (used by is-failed)
# 2	  "program is dead and /var/lock lock file exists"	unused
# 3	  "program is not running"	                        unit is not active
# 4	  "program or service status is unknown"	          no such unit
#
# usage: if (( get_service_status 'OpenSprinkler' == 1 )); then ...
############################################################################## 
function get_service_status
{

  if [ "$#" != "1" ]; then
    echo "'usage: get_service_status 'service_name'" >&2
    exit 1
  fi
    
  if [ ! $(service_file_exists "$1") == 0 ]; then
    return 4;
  fi
  
  sudo systemctl status "$1" &>/dev/null;
  local res=$?;
  echo $res;
}
function decode_service_status
{

  if [ "$#" != "1" ]; then
    echo "'usage: decode_service_status 'get_service_status_return_code'" >&2
    exit 1
  fi
  

  declare -a msg=("service is OK" "service is not failed / program not running" "program not running" "service not active / program not running" "no such service")
  echo ${msg[$1]}

}  
##############################################################################
# Function: service_is_registered 'service_name'
#
# Parameter: service_name e.g. 'homeassistant' or 'OpenSprinkler.service' 
#
# Return: true (0) if the service is registered, (1) otherwise
# usage: if service_is_registered 'home-assistant@homeassistant'; then ...
############################################################################## 
function service_is_registered
{
  if (( $# != 1 )) ; then
    echo "usage: service_is_registered 'service_name'" >&2
    exit 1;     # 1 = error 
  fi 
  
  if [ $(systemctl status $1 2>/dev/null | wc -l) -gt 0 ]; then
    return 0;     # 0 = true
  else
    return 1;     # 1 = false
  fi
}

##############################################################################
# Function: service_is_active 'service_name'
#
# Parameter: service_name e.g. 'homeassistant' or 'OpenSprinkler.service' 
#
# Return: true (0) if the service is active, (1) otherwise
# usage: if service_is_active 'home-assistant@homeassistant'; then ...
############################################################################## 
function service_is_active
{
  if (( $# != 1 )) ; then
    echo "usage: service_is_active 'service_name'" >&2
    exit 1;     # 1 = error
  fi 
  
  if (systemctl --quiet is-active $1); then
    return 0;     # 0 = true
  else
    return 1;     # 1 = false
  fi
}

#####################################################################################################################################################
##	MAIN OSPI 
####################################################################################################################################################

# Script version info
script_version='V2.2.1.100';  ## 2024-09-08
package="jq"
if [ $(dpkg-query -W -f='${Status}' ${package} 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
  echo "Package ${package} needs to be installed..."
  sudo apt-get install -y ${package};
fi
script_info_json=$(echo '{"script_info": "'"$(basename "$0") ${script_version}"'", "last_run_date_time": "'"$(date '+%a %b %d, %Y %H:%M:%S')"'"}' | jq .)

# Globals
run_as=$(whoami);
home_directory="${HOME}/";                              # e.g. /home/james/
current_woring_directory="$(pwd)/";                     # e.g. /share/OpenSprinkler-2.2.0.3/
start_up_directory="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/"; 
now="$(date '+%a %b %d, %Y %H:%M:%S')";                 # e.g. Thur Aug 29, 2024 14:56:07
services_directory='/etc/systemd/system/';              # All Directory Names Must End With '/'
executable_name='OpenSprinkler';
service_name="${executable_name}";
service_file_name="${service_name}.service";
target_directory="/usr/local/bin/${executable_name}/";     # All Directory Names Must End With '/'
compile_opensprinkler_flag='false';
install_opensprinkler_flag='false';
create_service_flag='false';
install_service_flag='false';
service_status_flag='false';
restart_service_flag='false';
skip_apt_update_flag='false'
verbose_flag='false';
script_execution_mode='none'; 
one_pass_complete_flag='false'; 

# Script version info
echo 
echo "Version: build.sh ${script_version}";

# This script must be run as james
if [[ $run_as != "james" ]]; then
  echo "Please run this script as james!";
  exit 0;
fi

# Process command line arguments
while getopts :cisydrx?: opt
do
    case "${opt}" in
        c) compile_opensprinkler_flag='true';;    
        i) install_opensprinkler_flag='true';;
        s) create_service_flag='true';;
        y) install_service_flag='true';;
        d) service_status_flag='true';;          
        r) restart_service_flag='true';;
        x) skip_apt_update_flag='true';;
        ?) echo "$usage";exit 0;;
        \?) printf "illegal option: -%s\n" "$OPTARG" >&2;echo "$usage" >&2;exit 1;;        
    esac
done



# Check for no command line options provided, if so select "interactive" mode
if [ "$#" == "0" ]; then
#if [[ $compile_opensprinkler_flag == "false" && $install_opensprinkler_flag == "false" && $create_service_flag == "false" && $install_service_flag == "false" && $service_status_flag == "false" && $restart_service_flag == "false" ]]; then
  script_execution_mode='interactive';
else
  script_execution_mode='command_line'
fi  


# Loop forever (until break is issued)
while true; do

  if [[ $script_execution_mode == "interactive" ]]; then
    if [[ $one_pass_complete_flag == "false" ]]; then
      echo
      echo "OpenSpinkler Executable:        ${start_up_directory%?}/${executable_name}"
      echo "OpenSpinkler Target Location:   ${target_directory%?}"
      echo "OpenSpinkler Service:           ${services_directory%?}/${service_file_name}"
      echo
    fi
    
    echo "Please select an option:"
    echo "  1) Compile OpenSprinkler${executable_name}"
    echo "  2) Install OpenSprinkler in ${target_directory%?}"
    echo "  3) Create OpenSprinkler.service ${service_file_name}"
    echo "  4) Install OpenSprinkler.service ${services_directory%?}/${service_file_name}"
    echo "  5) Current OpenSprinkler Service status ${service_file_name}"
    echo "  6) Restart OpenSprinkler Service ${service_file_name}"
    echo "  q) Exit"
    echo
    echo  
    while true; do
        read -p "Select [1,2,3,4,5,6,q]?" yn
        case $yn in
            [1]* ) compile_opensprinkler_flag='true';install_opensprinkler_flag='false';create_service_flag='false';install_service_flag='false';service_status_flag='false';restart_service_flag='false';break;;
            [2]* ) compile_opensprinkler_flag='false';install_opensprinkler_flag='true';create_service_flag='false';install_service_flag='false';service_status_flag='false';restart_service_flag='false';break;;
            [3]* ) compile_opensprinkler_flag='false';install_opensprinkler_flag='false';create_service_flag='true';install_service_flag='false';service_status_flag='false';restart_service_flag='false';break;;
            [4]* ) compile_opensprinkler_flag='false';install_opensprinkler_flag='false';create_service_flag='false';install_service_flag='true';service_status_flag='false';restart_service_flag='false';break;;
            [5]* ) compile_opensprinkler_flag='false';install_opensprinkler_flag='false';create_service_flag='false';install_service_flag='false';service_status_flag='true';restart_service_flag='false';break;; 
            [6]* ) compile_opensprinkler_flag='false';install_opensprinkler_flag='false';create_service_flag='false';install_service_flag='false';service_status_flag='false';restart_service_flag='true';break;;             
            [QqNn]* ) exit 0;;
            * ) exit 0;;
        esac
    done
  fi  
  
  # Debug info
  if [[ $verbose_flag == "true" && $one_pass_complete_flag == "false" ]]; then
    echo "Compile OpenSprinkler Flag: $compile_opensprinkler_flag";
    echo "Install OpenSprinkler Flag: $install_opensprinkler_flag";
    echo "Create Service Flag: $create_service_flag";
    echo "Install Service Flag: $install_service_flag";
    echo "Restart Service Flag: $restart_service_flag";  
    echo "Verbose Flag: $verbose_flag";
  fi


  ###########################################################################################################
  ## -c $compile_opensprinkler_flag == "true"
  ########################################################################################################### 
  if [[ $compile_opensprinkler_flag == "true" ]]; then
  
    echo "Compiling:  ${start_up_directory%?}/${executable_name}"
    
    if [[ $one_pass_complete_flag == "false" ]]; then
    
      if [[ $skip_apt_update_flag == "false" ]]; then
        ## Compilation Prerequisites  Compilation Prerequisites  Compilation Prerequisites  Compilation Prerequisites ##     
        sudo apt-get update
      fi  
        
      # Git update submodules
      # https://github.com/gilmaimon/TinyWebsockets
      # https://github.com/OpenThingsIO/OpenThings-Framework-Firmware-Library
      if git submodule status | grep --quiet '^-'; then
          echo "A git submodule is not initialized."
          git submodule update --recursive --init
      else
          echo "Updating submodules."
          git submodule update --recursive
      fi
      
      package="libmosquitto-dev"
      if [ $(dpkg-query -W -f='${Status}' ${package} 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
        echo "Package ${package} needs to be installed..."
        sudo apt-get install -y ${package};
      else	
        echo "Package \"${package}\" is already installed"	
      fi

      package="libssl-dev"
      if [ $(dpkg-query -W -f='${Status}' ${package} 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
        echo "Package ${package} needs to be installed..."
        sudo apt-get install -y ${package};
      else	
        echo "Package \"${package}\" is already installed"	
      fi

      package="libgpiod-dev"
      if [ $(dpkg-query -W -f='${Status}' ${package} 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
        echo "Package ${package} needs to be installed..."
        sudo apt-get install -y ${package};
      else	
        echo "Package \"${package}\" is already installed"	
      fi

      # Don't try to install raspi-gpio if it isn't available via apt-get [install via github https://github.com/RPi-Distro/raspi-gpio instead]
      package="raspi-gpio"
      if [ $(dpkg-query -W -f='${Status}' ${package} 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
        if sudo apt-get --simulate install ${package} 2>/dev/null; then  
          apt-get install -y ${package}
        fi
      else	
        echo "Package \"${package}\" is already installed"	
      fi
      # Check that raspi-gpio is actually installed 
      if ! command -v ${package} &> /dev/null; then
        echo "Command raspi-gpio is required but is NOT installed"
        echo "Install via github https://github.com/RPi-Distro/raspi-gpio instead"
        exit 0
      fi
    fi # end if $one_pass_complete_flag == "false"    
      

    USEGPIO=""
    GPIOLIB=""

    # Check for Raspberry Pi 5
    if [ -h "/sys/class/gpio/gpiochip512" ]; then
      echo "using libgpiod"
      USEGPIO="-DLIBGPIOD"
      GPIOLIB="-lgpiod"
    fi
    
    # Must use GPIOD when running on noble (24.04.1) or higher
    # See: https://github.com/OpenSprinkler/OpenSprinkler-Firmware/issues/250
    ubuntu_version=$(lsb_release -d 2>/dev/null | cut -d ' ' -f2)
    echo "Executing on OS: $ubuntu_version KERNAL: $(uname -r)"
    ubuntu_version_major=$(echo $ubuntu_version | cut -d '.' -f1)
    ubuntu_version_minor=$(echo $ubuntu_version | cut -d '.' -f2)
    ubuntu_version_rev=$(echo $ubuntu_version | cut -d '.' -f3)
    if [[ $(expr $ubuntu_version_major + 0) -ge 24 && $(expr $ubuntu_version_minor + 0) -ge 4 && $(expr $ubuntu_version_rev + 0) -ge 1 ]]; then
      echo "Newer kernal found: UBUNTU V$ubuntu_version_major.$ubuntu_version_minor.$ubuntu_version_rev or higher =>Using libgpiod instead of classic sysfs for GPIO"
      USEGPIO="-DLIBGPIOD"
      GPIOLIB="-lgpiod"
    else
      echo "Older kernal found: UBUNTU V$ubuntu_version_major.$ubuntu_version_minor.$ubuntu_version_rev =>Using classic sysfs for GPIO"      
    fi  

    #echo "Compiling ${executable_name} [OSPI] firmware using g++..."

    # V2.1.9.7
    # g++ -std=c++11 -Wno-psabi -o OpenSprinkler -DOSPI -I spdlog/include/ main.cpp OpenSprinkler.cpp program.cpp opensprinkler_server.cpp utils.cpp weather.cpp gpio.cpp etherport.cpp mqtt.cpp Descriptor.cpp DescriptorCollection.cpp UniqueObjectID.cpp Constants.cpp spdlog/build/libspdlog.a -lwiringPi -lpthread -lmosquitto
    
    # V2.2.0.3
    # g++ -std=c++14 -o OpenSprinkler -DOSPI main.cpp -I ../spdlog-1.10.0/include/ OpenSprinkler.cpp program.cpp opensprinkler_server.cpp utils.cpp weather.cpp gpio.cpp etherport.cpp mqtt.cpp ../spdlog-1.10.0/build/libspdlog.a -lpthread -lmosquitto
    
    # V2.2.1.0
    #ws=$(ls external/TinyWebsockets/tiny_websockets_lib/src/*.cpp)
    #otf=$(ls external/OpenThings-Framework-Firmware-Library/*.cpp)  
    #g++ -o OpenSprinkler -DOSPI $USEGPIO -DSMTP_OPENSSL $DEBUG -std=c++14 -include string.h main.cpp OpenSprinkler.cpp program.cpp opensprinkler_server.cpp utils.cpp weather.cpp gpio.cpp mqtt.cpp smtp.c -Iexternal/TinyWebsockets/tiny_websockets_lib/include $ws -Iexternal/OpenThings-Framework-Firmware-Library/ $otf -lpthread -lmosquitto -lssl -lcrypto $GPIOLIB

    # V2.2.1.100
    echo "Compiling ${executable_name} [OSPI] [SPDLOG V1.14.1] firmware using g++..."
    ws=$(ls external/TinyWebsockets/tiny_websockets_lib/src/*.cpp)
    otf=$(ls external/OpenThings-Framework-Firmware-Library/*.cpp)  
    g++ -o OpenSprinkler -DOSPI $USEGPIO -DSMTP_OPENSSL -std=c++14 -include string.h main.cpp OpenSprinkler.cpp program.cpp opensprinkler_server.cpp utils.cpp weather.cpp gpio.cpp mqtt.cpp smtp.c ../spdlog-1.14.1/build/libspdlog.a -I external/TinyWebsockets/tiny_websockets_lib/include $ws -I external/OpenThings-Framework-Firmware-Library/ $otf -I ../spdlog-1.14.1/include/ -lpthread -lmosquitto -lssl -lcrypto $GPIOLIB


    # $? will be 0 for a successful g++ compile
    if [ $? -ne 0 ]; then
      echo ">>> g++ compile error occured, exiting...!"
      exit 1;
    else
     echo "Successfully compiled OpenSprinkler"
    fi
    
    compile_opensprinkler_flag='false'
    
  fi  # end if $compile_opensprinkler_flag == "true"


  ###########################################################################################################
  ## -i $install_opensprinkler_flag == "true"
  ########################################################################################################### 
  if [[ $install_opensprinkler_flag == "true" ]]; then
  
    echo "Installing: ${start_up_directory%?}/${executable_name}"
    
    # Check if executable has been compiled
    if [[ ! -f "${start_up_directory%?}/${executable_name}" ]]; then
      echo "Executable file: ${start_up_directory%?}/${executable_name} does not exist!" 
    else
      # Check if target directory exists
      if [[ ! -d "${target_directory}" ]]; then
        echo "Target directory: ${target_directory} does not exist; creating...!"
        sudo mkdir "${target_directory}"
      fi

      if [[ -f "${services_directory%?}/${service_file_name}" ]]; then
        echo "Stopping ${service_file_name} ..."
        sudo systemctl stop "${service_file_name}"
      fi
      
      # Check if target directory and executable exists
      if [[ -f "${target_directory%?}/${executable_name}" ]]; then
        echo "Target executable: ${target_directory%?}/${executable_name} already exists!"
        # Save old service file to current dir
        echo "Saving old Executable File to : ${target_directory%?}/${executable_name}.last";
        if [[ -f "${target_directory%?}/${executable_name}.last" ]]; then        
          sudo rm "${target_directory%?}/${executable_name}.last" 
        fi
        sudo mv -v "${target_directory%?}/${executable_name}" "${target_directory%?}/${executable_name}.last"
      fi
      
      # Copy the local executable file to /usr/bin/local/OpenSprinkler/
      sudo cp -v "${start_up_directory%?}/${executable_name}" "${target_directory}"
      
      # Make file owned by root && executable
      sudo chown root:root "${target_directory%?}/${executable_name}"
	    sudo chmod +x "${target_directory%?}/${executable_name}"

        
    fi  # end if executable does not exist locally
    
    install_opensprinkler_flag='false'
    
  fi  # end if $install_opensprinkler_flag == "true"
  
  
  ###########################################################################################################
  ## -s $create_service_flag == "true"
  ########################################################################################################### 
  if [[ $create_service_flag == "true" ]]; then

    if [[ -f "${start_up_directory%?}/${service_file_name}" ]]; then
      rm "${start_up_directory%?}/${service_file_name}"
    fi


    if [[ ! -f "${start_up_directory%?}/${service_file_name}" ]]; then
      echo "Creating new service file: ${start_up_directory%?}/${service_file_name}..." 
      sudo echo "" > "${start_up_directory%?}/${service_file_name}";
      sudo echo "[Unit]" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "Description=OpenSprinkler Service V1.3 [OpenSprinkler V2.2.1.100 Sept 08, 2024]" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "After=network-online.target" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "Wants=network-online.target" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "[Service]" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "Type=simple" >> "${start_up_directory%?}/${service_file_name}"; 
      sudo echo "User=root" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "ExecStart=${target_directory%?}/${executable_name}" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "StandardOutput=null" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "[Install]" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "WantedBy=multi-user.target" >> "${start_up_directory%?}/${service_file_name}";
      sudo echo "" >> "${start_up_directory%?}/${service_file_name}";
      echo "Successfully created ${start_up_directory%?}/${service_file_name}"
    fi  

    create_service_flag='false'
    
  fi  # end if $create_service_flag == "true"
  

  ###########################################################################################################
  ## -y $install_service_flag == "true"
  ########################################################################################################### 
  if [[ $install_service_flag == "true" ]]; then

    echo "Installing new Service File to : ${services_directory%?}/${service_file_name}";

    # Get current service status... return newly installed service to the same state on exit if possible
    service_state=$(get_service_status "${service_file_name}")
    echo "Current service state: [$service_state]=> "$(decode_service_status "$service_state")

    if [[ ! -f "${start_up_directory%?}/${service_file_name}" ]]; then
      echo "Service file: ${start_up_directory%?}/${service_file_name} does not exist!" 
    else  # New service file has been created
      if [[ -f "${services_directory%?}/${service_file_name}" ]]; then
        sudo systemctl stop "${service_file_name}"
        sudo systemctl disable "${service_file_name}"
        # Save old service file to current dir
        echo "Saving old Service File to : ${start_up_directory%?}/${service_file_name}.last"; 
        sudo mv -v "${services_directory%?}/${service_file_name}" "${start_up_directory%?}/${service_file_name}.last"
      fi  # end if service already exists 
    
      echo "Installing new Service File: ${services_directory%?}/${service_file_name}"; 
      
      # Copy service file to /etc/systemd/system/
      sudo cp -v "${start_up_directory%?}/${service_file_name}" "${services_directory}";
      sudo chown root:root "${services_directory%?}/${service_file_name}"; 

      # Reload systemd
      sudo systemctl daemon-reload

      # Enable the new service unit ONLY if it wasn't previously disabled
      #if [[ $original_service_state != "3" && $(get_service_status "${service_file_name}") != "4" ]]; then
        # Enable the service
        sudo systemctl enable "${service_file_name}"
      #fi  
      
      # Start the new service unit ONLY if it was previously running -OR- stopped
      if [[ ( $original_service_state == "0" || $original_service_state == "3" ) && $(get_service_status "${service_file_name}") != "4" ]]; then
        # Restart the service
        echo "Restarting service: ${service_file_name}";
        sudo systemctl restart "${service_file_name}"
      fi
      
      echo "Successfully installed ${service_file_name}"
    fi  # end if new service file exists 

    install_service_flag='false'
    
  fi  # end if $install_service_flag == "true"


  ###########################################################################################################
  ## -d $service_status_flag == "true"
  ########################################################################################################### 
  if [[ $service_status_flag == "true" ]]; then

    echo "Getting current service status: ${service_file_name}";
    
    sudo systemctl status "${service_file_name}"
    
    service_status_flag='false'
        
  fi  # end if $service_status_flag == "true"    
  

  ###########################################################################################################
  ## -r $restart_service_flag == "true"
  ########################################################################################################### 
  if [[ $restart_service_flag == "true" ]]; then

    echo "Restarting service: ${service_file_name}";
    
    # 4 == unit file not loaded or service file does not exist
    if [[ $(get_service_status "${service_file_name}") != "4" ]]; then   
      # Restart the service
      sudo systemctl restart "${service_file_name}"
    
      service_state=$(get_service_status "${service_file_name}")
      echo "Current service state: [$service_state]=> "$(decode_service_status "$service_state")
    
    else
      echo "Service ${service_file_name} does not exist!"
    fi

    
    restart_service_flag='false'
        
  fi  # end if $restart_service_flag == "true"    
  
  


  ###########################################################################################################
  ## Done
  ###########################################################################################################

  # Indicate that at least one pass has been completed and therefore reduce printing debug info 
  one_pass_complete_flag='true'
  
  if [[ $script_execution_mode == "command_line" ]]; then
    echo "Done."
    #exit 0;
    break;
  fi
  
done

  
##########################################################################################################
### EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF  EOF ###
##########################################################################################################

