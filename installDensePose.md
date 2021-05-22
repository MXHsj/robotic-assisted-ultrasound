## DensePose Installation Guide
- author: Xihan Ma
- date: 05-21-2021
- reference: [DensePose Colab notebook](https://colab.research.google.com/github/tugstugi/dl-colab-notebooks/blob/master/notebooks/DensePose.ipynb)
---

### Prerequisites
1. Ubuntu 18.04
2. NVIDIA GPU & appropriate driver
3. CUDA version >= 10.0
4. cuDNN version >= 7.0

### Steps
1. install Anaconda Python 2.7

    under home directory:
    ```sh
    wget -q https://repo.anaconda.com/archive/Anaconda2-2019.03-Linux-x86_64.sh
    chmod +x Anaconda2-2019.03-Linux-x86_64.sh
    bash ./Anaconda2-2019.03-Linux-x86_64.sh -b -f -p ~/anaconda2
    ```

2. activate conda virtual environment

    under home directory:
    ```sh
    source ~/anaconda2/bin/activate
    ```

3. install dependencies

    under home directory:
    ```sh
    # PyTorch
    conda install -y pyyaml=3.12
    conda install -y mkl-include
    conda install -y pytorch=1.0.1 torchvision cudatoolkit=10.0 -c pytorch
    ln -s ~/anaconda2/lib/python2.7/site-packages/torch/lib/ ~/anaconda2/lib/python2.7/site-packages/
    # GCC 4.9
    conda install -y -c serge-sans-paille gcc_49
    ln -fs ~/anaconda2/lib/libmpfr.so ~/anaconda2/lib/libmpfr.so.4
    # protobuf 3.5
    conda install -y protobuf=3.5
    # pycocotools
    conda install -y -c conda-forge pycocotools
    # some missing dependencies
    pip install opencv-python==4.0.0.21 memory_profiler
    # some headers from the pytorch source
    git clone -q --depth 1 --recursive -b v1.0.1 https://github.com/pytorch/pytorch
    ```

4. Clone & build DensePose project

    under home directory:
    ```sh
    git clone -q --depth 1 https://github.com/facebookresearch/DensePose.git
    cd DensePose
    pip install -q -r requirements.txt
    ```
    overwrite **CMakeLists.txt** with:
    ```cmake
    cmake_minimum_required(VERSION 2.8.12 FATAL_ERROR)
    set(Caffe2_DIR "~/anaconda2/lib/python2.7/site-packages/torch/share/cmake/Caffe2/")
    find_package(Caffe2 REQUIRED)

    include_directories("~/anaconda2/lib/python2.7/site-packages/torch/lib/include")
    include_directories("~/anaconda2/include")
    include_directories("~/pytorch")

    add_library(libprotobuf STATIC IMPORTED)
    set(PROTOBUF_LIB "~/anaconda2/lib/libprotobuf.a")
    set_property(TARGET libprotobuf PROPERTY IMPORTED_LOCATION "${PROTOBUF_LIB}")

    if (${CAFFE2_VERSION} VERSION_LESS 0.8.2)
    # Pre-0.8.2 caffe2 does not have proper interface libraries set up, so we
    # will rely on the old path.
    message(WARNING
        "You are using an older version of Caffe2 (version " ${CAFFE2_VERSION}
        "). Please consider moving to a newer version.")
    include(cmake/legacy/legacymake.cmake)
    return()
    endif()

    # Add compiler flags.
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -fPIC -Wno-narrowing")

    # Print configuration summary.
    include(cmake/Summary.cmake)
    detectron_print_config_summary()

    # Collect custom ops sources.
    file(GLOB CUSTOM_OPS_CPU_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/detectron/ops/*.cc)
    file(GLOB CUSTOM_OPS_GPU_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/detectron/ops/*.cu)

    # Install custom CPU ops lib.
    add_library(
        caffe2_detectron_custom_ops SHARED
        ${CUSTOM_OPS_CPU_SRCS})

    target_link_libraries(caffe2_detectron_custom_ops caffe2_library libprotobuf)
    install(TARGETS caffe2_detectron_custom_ops DESTINATION lib)

    # Install custom GPU ops lib, if gpu is present.
    if (CAFFE2_USE_CUDA OR CAFFE2_FOUND_CUDA)
    # Additional -I prefix is required for CMake versions before commit (< 3.7):
    # https://github.com/Kitware/CMake/commit/7ded655f7ba82ea72a82d0555449f2df5ef38594
    list(APPEND CUDA_INCLUDE_DIRS -I${CAFFE2_INCLUDE_DIRS})
    CUDA_ADD_LIBRARY(
        caffe2_detectron_custom_ops_gpu SHARED
        ${CUSTOM_OPS_CPU_SRCS}
        ${CUSTOM_OPS_GPU_SRCS})

    target_link_libraries(caffe2_detectron_custom_ops_gpu caffe2_gpu_library libprotobuf)
    install(TARGETS caffe2_detectron_custom_ops_gpu DESTINATION lib)
    endif()
    ```
    build DensePose
    ```sh
    make
    make ops
    ```

5. Test DensePose
   
   under home directory:
   ```sh
   cd DensePose/DensePoseData && bash get_densepose_uv.sh
   source ~/anaconda2/bin/activate
   python2 tools/infer_simple.py \
    --cfg configs/DensePose_ResNet101_FPN_s1x-e2e.yaml \
    --output-dir DensePoseData/infer_out/ \
    --image-ext jpg \
    --wts https://dl.fbaipublicfiles.com/densepose/DensePose_ResNet101_FPN_s1x-e2e.pkl \
    DensePoseData/demo_data/demo_im.jpg
   ```
   note:
   pretrained weight can be downloaded ahead, then put directory to **DensePose_ResNet101_FPN_s1x-e2e.pkl** to the **--wts** argument. This will save the downloading time everytime running DensePose

### Debugging
1. if cannot find cudnn version, find cudnn version in **cudnn_version.h** instead of **cudnn.h**
2. if GPU memory <= 4GB, try crop the input image in square
3. 
