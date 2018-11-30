# Setting up autostart for the ihmc-logger


Edit ihmc-logger.service and adjust the home folder in ExecStart as well as the User and Group to the desired user/group. Then, run the following commands

```
sudo cp ihmc-logger.service /etc/systemd/system
sudo systemctl enable ihmc-logger 

```

