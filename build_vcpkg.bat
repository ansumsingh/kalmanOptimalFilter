REM install packages in vcpkg

pushd vendor\vcpkg

IF NOT EXIST vcpkg.exe bootstrap-vcpkg.bat 


vcpkg.exe integrate install

vcpkg.exe install eigen3 catch2 fmt --triplet x64-windows

popd