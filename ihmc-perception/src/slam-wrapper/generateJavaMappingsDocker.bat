@echo on

docker run --rm --volume %~dp0:C:\slam-wrapper --workdir C:\slam-wrapper ihmcrobotics/gtsam-windows-build:0.1 powershell.exe C:\slam-wrapper\generateJavaMappings.bat