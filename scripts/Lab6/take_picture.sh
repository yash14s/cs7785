#!/bin/bash

for i in {1..110}
do
	raspistill -e png -o $i.png -w 410 -h 308
	read -p "Hit enter to take next picture"
done
