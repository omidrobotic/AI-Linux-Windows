#!/bin/bash
echo "this code write mohammadhossein zolfaghari from omid robotic team"
echo "running......."
pwd
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done

cd ../..
rm -r build
mkdir build
cd build
cmake ..
make
