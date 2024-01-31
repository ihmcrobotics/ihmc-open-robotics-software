@echo on

docker run --rm --volume %~dp0:C:\promp --workdir C:\promp ihmcrobotics/windows-build:0.1 powershell.exe C:\promp\generateJavaMappings.bat