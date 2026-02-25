echo "Installing Ceres Solver..."
sudo apt update && sudo apt install -y cmake build-essential libgoogle-glog-dev libgflags-dev libatlas-base-dev libsuitesparse-dev
git clone --recurse-submodules https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make -j$(nproc)
sudo make install
cd ../../
rm -rf ceres-solver
