#!/usr/bin/env python

import time
from datetime import datetime

input_time = '2017_10_13_18_08_15_774'
print 'input_time ', input_time

pattern = '%Y_%m_%d_%H_%M_%S_%f'
epoch = (datetime.strptime(input_time, pattern) - datetime(1970,1,1)).total_seconds()
print 'epoch %.3f' % epoch

output_time = datetime.fromtimestamp(epoch).strftime(pattern)[:-3]

print 'output_time', output_time
