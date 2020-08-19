mkdir build

cmake.exe -S test -B build -DCMAKE_TOOLCHAIN_FILE=D:/kalmanOptimalFilter/vendor/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows
cmake.exe --build build


REM ctest --build-and-test test build --build-generator "Visual Studio 16 2019"