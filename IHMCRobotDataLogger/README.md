IHMC Robot Data Logger
======================

###### Deploy

Set Gradle properties `deployLoggerUser`, `deployLoggerPassword`, `deployLoggerHost`.

Run `deployLogger`.

###### Set To Run On Boot

Write to `/etc/systemd/system/logger.service`:
```
[Unit]
Description=IHMC Logger
After=network.target

[Service]
User=<user>
ExecStart=/bin/sh /root/runVelodyneROSTopic.sh

[Install]
WantedBy=multi-user.target
```

`sudo systemctl daemon-reload`
`sudo systemctl enable logger`
