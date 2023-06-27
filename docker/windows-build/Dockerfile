# Version 0.1

# Provides a Windows build container with VS 2019, CMake, Eigen 3.4.0, JDK 17

# Use ltsc2019 with dotnet 4.8 preinstalled - required for Chocolatey
# If we try to install dotnet 4.8 ourselves, it requires a reboot during the docker build process,
# which complicates things quite a bit and we'd have to a multi-stage build or something
FROM mcr.microsoft.com/dotnet/framework/sdk:4.8-windowsservercore-ltsc2019

SHELL ["powershell.exe"]

# Install chcolatey
RUN Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))

RUN choco install visualstudio2022-workload-vctools -y

RUN choco install cmake -y

RUN setx PATH '%PATH%;C:\Program Files\CMake\bin'

# Install Eigen 3.4.0
RUN curl https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip -o eigen-3.4.0.zip
RUN tar -xvf eigen-3.4.0.zip
RUN cd eigen-3.4.0; mkdir build; cd build; cmake ..; cmake --build . --target install

# Install JDK 17
RUN choco install temurin17 -y

ENTRYPOINT ["powershell.exe"]
