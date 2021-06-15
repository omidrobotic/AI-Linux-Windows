# AI-Linux-Windos-2021
in this code you can run in linux and windows 

* ### IN Linux


you need install <strong>googleProtobuf</strong> from:

https://github.com/protocolbuffers/protobuf/blob/master/src/README.md

download the repository

> `wget https://github.com/omidrobotic/AI-Linux-Windos-2021.git` 

or

> `git clon https://github.com/omidrobotic/AI-Linux-Windos-2021.git`

then go to located file

> `cd ai/Protobuf`
  
  and run
  
> `chmod 755 auto.sh`
  
  
then install <strong>Grsim</strong> from

https://github.com/RoboCup-SSL/grSim

run this cod from your <strong>Grsim</strong> System 


then install <strong>OpenGL and GLUT</strong>

update 
> `sudo apt update`

install librarys
> `sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev`


your run code system or if you use tow system run code in grsim system
<italic>
  
  * if one system:
    > hostname -I  
  
 Or 
 
  * if use tow system:
    > ifconfig
  
  see yor local IP address
  
  go to switch.h in ai and set IP address
  
  For example:
  > `#define GROUP_ADDR_SEND_GRSIM_COMMAND	"192.168.0.2"`
  
go to the code runner system and run:
> `./auto.sh`


