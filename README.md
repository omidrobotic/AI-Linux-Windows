# AI-Linux-Windos-2021
in this code you can run in linux and windows 

* ### IN Linux


you need install <strong>googleProtobuf</strong> from:

https://github.com/protocolbuffers/protobuf/blob/master/src/README.md

download the repository

> `wget https://github.com/omidrobotic/AI-Linux-Windos-2021.git` 

or

> `git clone https://github.com/omidrobotic/AI-Linux-Windos-2021.git`

then go to located file

> `cd ai/Protobuf`
  
  and run
  
> `chmod 755 auto.sh`
  
  

then install <strong>OpenGL and GLUT</strong>

update 
> `sudo apt update`

install librarys
> `sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev`

https://user.xmission.com/~nate/glut.html


your run code system or if you use tow system run code in simulator system
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
  
### You can run code in Real setup in Omid ERforce and Grsim Simulation

 ##### *fix switch simulator and mode
  
* #### run in <strong>Grsim</strong>:
  
then install <strong>Grsim</strong> from

https://github.com/RoboCup-SSL/grSim

run this cod from your <strong>Grsim</strong> System 

go to the code runner system and run:
> `./auto.sh`

 type 'y' to run
  
  next command 
  
 > `make`
  
  finaly
  
  >`./omid'

  
  
  * ### IN Linux
  https://www.youtube.com/watch?v=ES_GI-lmhEU
  https://www.youtube.com/watch?v=8_X5Iq9niDE
