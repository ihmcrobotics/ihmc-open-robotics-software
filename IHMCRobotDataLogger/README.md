IHMC Robot Data Logger
======================

###### Deploy

Set Gradle properties `deployLoggerUser`, `deployLoggerPassword`, `deployLoggerHost`.

Run `deployLogger`.

###### Configure Logger

The logger can be configured using `[user.home]/.ihmc/IHMCLoggerConfiguration.ini` or trough Java System properties. 


The format for  `IHMCLoggerConfiguration.ini` is:
```
loggerNetwork=[IP or hostname of the interface to bind to]
camerasToCapture=[Comma seperated list of cameras to capture, optional]
```

The following java system properties are available. These are considered VM arguments, so add these before your main class decleration or -jar option when calling your application. 
```
-Dihmc.loggerNetwork=[IP or hostname of the interface to bind to]
-Dihmc.camerasToCapture=[Comma seperated list of cameras to capture, optional]
```

Java properties will override the configuration file.

###### Set To Run On Boot

Write to `/etc/systemd/system/logger.service`:
```
[Unit]
Description=IHMC Logger
After=network.target

[Service]
User=<user>
ExecStart=/home/user/IHMCLogger/bin/IHMCLogger

[Install]
WantedBy=multi-user.target
```

`sudo systemctl daemon-reload`

`sudo systemctl enable logger`
