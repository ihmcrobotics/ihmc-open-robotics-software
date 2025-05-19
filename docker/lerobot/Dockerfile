FROM ihmcrobotics/nvidia:0.6

USER root

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    cmake \
    g++ \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Set the installation directory for Miniconda
ENV CONDA_DIR /opt/conda

 # Download and install Miniconda
 RUN wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
     /bin/bash ~/miniconda.sh -b -p $CONDA_DIR && \
     rm ~/miniconda.sh

 # Add conda to the PATH
 ENV PATH=$CONDA_DIR/bin:$PATH

USER robotlab
