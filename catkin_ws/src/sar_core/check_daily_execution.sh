#!/bin/bash
# Script to check if game has been played and logs date of playing

PLAYED_TODAY=0
file="daily_execution_log.txt"
while IFS= read -r line
do
        # display $line or do somthing with $line
    if [ "$line" = $(date +%Y-%m-%d) ]
	then
	    PLAYED_TODAY=1		
	fi
done <"$file"

if [ $PLAYED_TODAY = 1 ]
then 
	echo "You've already played today! Play again tomorrow!"
else
	echo "Welcome! Enjoy playing!"
	echo $(date +%Y-%m-%d) >> "$file"
fi

