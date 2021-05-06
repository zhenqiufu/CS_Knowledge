#########################################################################
# File Name: testAll.sh
# Author: FU Zhenqiu
# mail: fuzhenqiu0810@gmail.com
# Created Time: 2021年04月21日 星期三 19时49分40秒
#########################################################################
#!/bin/bash
echo '*************************'
echo RUN TEST FOR ALL THE CODE
echo '*************************'

# test for exercise 1-23
echo 1-23 NO TEST CASE

# ./exercise_1-23
# if [ `grep "error" file.txt  &>> error.txt` ] ;then
#    echo 'find'
# else
#   echo 'no error find in file.txt'
# fi

grep "error" file.txt > /dev/null
if [ $? -eq 0 ]; then
    echo "Found!"
else
    echo "Not found!"
fi
