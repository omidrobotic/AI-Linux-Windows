#!/bin/bash
echo "this code write mohammadhossein zolfaghari from omid robotic team"
echo "running......."
pwd





echo "you want run code in erforce kachal or grsim simulation?"
echo "type y to use Grsim and n to use ER-force ('y/n')?"
read simulation;

cd  ai/Protobuf


echo "remove erforce"
cd ER-force
rm *.cc
rm *.h

cd vision
rm *.cc
rm *.h

cd ../erforce
rm *.cc
rm *.h


echo "remove grsim"
cd ../../Grsim
rm *.cc
rm *.h

cd ../Refree
rm *.cc
rm *.h

cd ../Vision
rm *.cc
rm *.h


cd ..


if [[($simulation == "y")]]; then
echo "grsim"
cd Grsim
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done


cd ../Vision
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done


cd ../Refree
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done

cd ..


else
echo "ER-force"



cd Refree
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done
cd ..


cd ER-force #/vision
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done

#cd ../erforce
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done

#cd ..
file=$(find -name "*.proto")
for val in $file;do
     echo $val
     protoc -I=. --cpp_out=. $val
done

cd ..

fi



cd ../..
rm -r build
mkdir build
cd build
cmake ..

