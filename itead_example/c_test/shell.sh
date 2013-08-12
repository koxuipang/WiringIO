#!/bin/sh

echo "intput data"
echo $1
echo $2
echo "building..."
gcc -o $1 $2 $3 $4 $5 $6 $7 $8 $9 -lwiringIO -lpthread -lm
echo "ok!"

