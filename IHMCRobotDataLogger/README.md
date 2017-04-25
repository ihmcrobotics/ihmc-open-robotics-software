IHMC Robot Data Logger
======================

###### Deploy

Set Gradle properties `deployLoggerUser`, `deployLoggerPassword`, `deployLoggerHost`.

Run `deployLogger`.

###### Configure Network

Ensure file contains line in `~/.ihmc/IHMCNetworkParameters.ini`:
```
logger=<logger_ip>
```

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
