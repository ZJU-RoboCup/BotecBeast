#!/bin/sh
echo formating !!!
./clang-format-all src
./clang-format-all include
array=`git diff-index --name-only HEAD`
for name in ${array} 
do
echo ${name}
git add ${name}
done
echo add formated file
