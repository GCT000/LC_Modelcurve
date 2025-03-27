# 使用Ubuntu基础镜像
FROM ubuntu:20.04 as builder

# 避免交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 安装基本工具和编译环境
RUN apt-get update && apt-get install -y  --no-install-recommends \
    build-essential \
    cmake \
    git \
    libgtest-dev \
    libeigen3-dev \
    libopencv-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    && rm -rf /var/lib/apt/lists/*

# 设置工作目录
WORKDIR /app

# 复制项目文件
COPY . .

# 安装TBB（替换原有的libtbb-dev安装）
RUN cd /app/thirdparty && \
    tar xzf oneTBB-2020.3.tar.gz && \
    cd oneTBB-2020.3 && \
    make tbb CXXFLAGS="-O2 -D__TBB_USE_ADDRESS_SANITIZER=0" && \
    # 安装头文件
    cp -r include/tbb /usr/local/include/ && \
    # 安装库文件
    cp build/*_release/*.so* /usr/local/lib/ && \
    # 更新库缓存
    ldconfig

# 安装gflags
RUN cd /app/thirdparty && \
    tar zxf gflags-2.2.2.tar.gz && \
    cd gflags-2.2.2 && \
    mkdir build && cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
    make -j4 && \
    make install

# 安装glog
RUN cd /app/thirdparty && \
    tar zxf glog-0.6.0.tar.gz && \
    cd glog-0.6.0 && \
    mkdir build && cd build && \
    cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON .. && \
    make -j4 && \
    make install

# 安装Ceres（如果需要特定版本）
RUN cd /app/thirdparty && \
    tar xzf ceres-solver.tar.gz && \
    cd ceres-solver && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install

# 编译项目
RUN rm -rf build && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4

FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# 安装系统提供的运行时依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    libopencv-core4.2 \
    libopencv-imgproc4.2 \
    libopencv-calib3d4.2 \
    libopencv-video4.2 \
    libeigen3-dev \
    libblas3 \
    liblapack3 \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    libpcl-io1.10 \
    libpcl-filters1.10 \
    libpcl-kdtree1.10 \
    libpcl-segmentation1.10 \
    libpcl-visualization1.10 \
    && rm -rf /var/lib/apt/lists/*

# 复制自定义构建的依赖（Ceres等）
COPY --from=builder \
    /usr/local/lib/lib*.so* \
    /usr/local/lib/

# 复制项目构建结果
COPY --from=builder \
    /app/bin/test \
    /app/lib/libcurveModel_lib.so \
    /app/

ENV LD_LIBRARY_PATH=/usr/local/lib:/app:$LD_LIBRARY_PATH

# 设置工作目录
WORKDIR /app