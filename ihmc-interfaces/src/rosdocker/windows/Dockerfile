FROM microsoft/windowsservercore:1803 as ros2_build_base_win

# I guess this makes the image smaller
ENV ChocolateyUseWindowsCompression false

# Install chocolatey
RUN @"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass \
    -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && \
    SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"

# From https://github.com/ros2/ros2/wiki/Windows-Install-Binary
# and  https://github.com/ros2/ros2/wiki/Windows-Development-Setup
RUN choco install -y python git cmake patch 7zip curl

RUN choco install -y visualstudio2017community visualstudio2017-workload-nativedesktop

#RUN pip install vcstool
#RUN pip install -U colcon-common-extensions
#RUN pip install -U setuptools pip
#RUN pip install catkin_pkg EmPy pyparsing pyyaml
#RUN pip install pytest coverage mock
#pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions \
#    flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pep8 pydocstyle
#RUN choco install -y cppcheck
#
### wait for vs_installer.exe, vs_installerservice.exe
### or vs_installershell.exe because choco doesn't
##RUN powershell -NoProfile -InputFormat None -Command \
##    choco install -y git 7zip; \
##    choco install -y visualstudio2017community visualstudio2017-workload-nativedesktop;
#
#ARG ROS_VERSION_ARG
#
#WORKDIR C:/
#RUN md dev/ros2
#WORKDIR C:/dev/ros2

ENTRYPOINT ["C:\\Windows\\System32\\cmd.exe", "/k", "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat"]
