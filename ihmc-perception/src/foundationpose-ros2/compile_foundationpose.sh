export ROOT=/root/foundationpose-ros2

# Clone FoundationPose repo
git clone https://github.com/ArghyaChatterjee/FoundationPose.git

# Install PyTorch and basic Python dependencies
pip install torchvision==0.16.0+cu121 torchaudio==2.1.0 torch==2.1.0+cu121 --index-url https://download.pytorch.org/whl/cu121 && \
pip install -r requirements.txt && \
pip install setuptools==68.2.2


# Build & Install pybind11
cd $ROOT/FoundationPose && \
git clone https://github.com/pybind/pybind11 && cd pybind11 && git checkout v2.10.0 && \
mkdir build && cd build && \
cmake .. -DCMAKE_BUILD_TYPE=Release -DPYBIND11_INSTALL=ON -DPYBIND11_TEST=OFF && \
make -j4 && make install

# Build & Install Eigen
cd $ROOT/FoundationPose && \
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
tar xvzf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz && \
cd eigen-3.4.0 && mkdir build && cd build && \
cmake .. && make -j4 && make install

# Install nvdiffrast
cd $ROOT/FoundationPose && \
git clone https://github.com/NVlabs/nvdiffrast && cd nvdiffrast && \
pip install .

# Build mycpp
cd $ROOT/FoundationPose/mycpp && \
rm -rf build && mkdir build && cd build && \
cmake .. && make -j4

# Build & Install mycuda
cd $ROOT/FoundationPose/bundlesdf/mycuda && \
rm -rf build *egg* *.so && \
export TORCH_CUDA_ARCH_LIST="8.0;8.6;8.9+PTX" && \
python3 -m pip install -e .
