FROM microsoft/windowsservercore:1803 as ros2_build_base_win

# I guess this makes the image smaller
ENV ChocolateyUseWindowsCompression false

# Install chocolatey
RUN @"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass \
    -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && \
    SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"

# From https://github.com/ros2/ros2/wiki/Windows-Install-Binary

# Install command line utilities
RUN choco install -y python --version 3.6.5
RUN choco install -y cmake curl 7zip patch git which
RUN setx PATH "%PATH%;C:\Python37\Scripts"

# Install tee (useful to see status while logging to file)
WORKDIR C:/Users/ContainerUser/Downloads
RUN curl -skL https://downloads.sourceforge.net/project/unxutils/unxutils/current/UnxUtils.zip -o UnxUtils.zip
RUN 7z x UnxUtils.zip -y -oUnxUtils
RUN md C:\tee
RUN copy UnxUtils\usr\local\wbin\tee.exe C:\tee\tee.exe
RUN setx PATH "%PATH%;C:\tee"

# Install OpenSSL
WORKDIR C:/Users/ContainerUser/Downloads
RUN curl -sk https://slproweb.com/download/Win64OpenSSL-1_0_2o.exe -o Win64OpenSSL.exe
RUN Win64OpenSSL.exe /VERYSILENT
RUN setx -m OPENSSL_CONF C:\OpenSSL-Win64\bin\openssl.cfg
RUN setx PATH "%PATH%;C:\OpenSSL-Win64\bin"

# Install Visual Studio
RUN choco install -y visualstudio2017community visualstudio2017-workload-nativedesktop

# Install OpenCV
WORKDIR C:/Users/ContainerUser/Downloads
RUN curl -skL https://github.com/ros2/ros2/releases/download/opencv-archives/opencv-3.4.1-vc15.VS2017.zip -o opencv.zip
RUN dir
RUN 7z x opencv.zip -y -oc:
RUN setx -m OpenCV_DIR C:\opencv
RUN setx PATH "%PATH%;C:\opencv\x64\vc15\bin"

# Install ROS choco dependencies
WORKDIR C:/Users/ContainerUser/Downloads
RUN curl -skL https://github.com/ros2/choco-packages/releases/download/2018-06-12-1/asio.1.12.1.nupkg -o asio.1.12.1.nupkg
RUN curl -skL https://github.com/ros2/choco-packages/releases/download/2018-06-12-1/eigen.3.3.4.nupkg -o eigen.3.3.4.nupkg
RUN curl -skL https://github.com/ros2/choco-packages/releases/download/2018-06-12-1/tinyxml-usestl.2.6.2.nupkg -o tinyxml-usestl.2.6.2.nupkg
RUN curl -skL https://github.com/ros2/choco-packages/releases/download/2018-06-12-1/tinyxml2.6.0.0.nupkg -o tinyxml2.6.0.0.nupkg
RUN choco install -y -s C:/Users/ContainerUser/Downloads asio eigen tinyxml-usestl tinyxml2

# Install python dependencies
RUN python -m pip install --upgrade setuptools pip
RUN pip install --upgrade catkin_pkg empy pyparsing pyyaml

# From https://github.com/ros2/ros2/wiki/Windows-Development-Setup

# Additional Prerequisites
RUN setx PATH "%PATH%;C:\Program Files\Git\cmd;C:\Program Files\CMake\bin"

# Installing Developer Tools
RUN pip install vcstool
RUN pip install --upgrade colcon-common-extensions

# Install dependencies
RUN pip install pytest coverage mock
RUN pip install flake8 flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions
RUN pip install flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes pep8 pydocstyle
RUN choco install -y cppcheck
RUN setx PATH "%PATH%;C:\Program Files\Cppcheck"

# Skip Install Qt5

# Getting the Source Code
ARG ROS_DISTRO
ENV ROS_DISTRO ${ROS_DISTRO}
WORKDIR C:/
RUN md dev\ros2\src
WORKDIR C:/dev/ros2
RUN curl -sk https://raw.githubusercontent.com/ros2/ros2/release-%ROS_DISTRO%/ros2.repos -o ros2.repos
RUN vcs import src < ros2.repos

# Building the ROS 2 Code
SHELL ["C:\\Windows\\System32\\cmd.exe", "/c", "\"C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat\" && "]
RUN colcon build --merge-install --packages-skip rviz

WORKDIR C:/dev/ros2
ENTRYPOINT ["C:\\Windows\\System32\\cmd.exe", "/k", "C:\\Program Files (x86)\\Microsoft Visual Studio\\2017\\Community\\VC\\Auxiliary\\Build\\vcvars64.bat"]
#ENTRYPOINT ["C:\\Windows\\System32\\cmd.exe"]
