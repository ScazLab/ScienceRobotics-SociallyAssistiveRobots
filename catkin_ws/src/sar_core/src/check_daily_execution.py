#!/usr/bin/env python

import json
import os.path
import time

session_data_filepath = '/home/sar/catkin_ws/src/sar_core/logs/session_data.json'


if os.path.isfile(session_data_filepath) == False:
    print "good"
    exit()

with open(session_data_filepath) as session_data_file:
    session_data_list = json.load(session_data_file)['sessions']
    # print session_data_list
    last_day_number = len(session_data_list)
    for daily_info in session_data_list:
        _day = daily_info['day']
        if _day == last_day_number:
            _date_time = daily_info['system_down_time'].encode('ascii','ignore')
            _date_time = _date_time.replace('  ', ' ')
            _breakdown_date_time = _date_time.split(' ')
            # print _day
            # print _date_time
            # print _breakdown_date_time
            _now = now = time.strftime("%c")
            # print _now
            _breakdown_now = _now.split(' ')
            # print _breakdown_now
            if _breakdown_date_time[1] == _breakdown_now[1] and _breakdown_date_time[2] == _breakdown_now[2]:
                # already played the application today
                print "bad"
            else:
                print "good"
            break
