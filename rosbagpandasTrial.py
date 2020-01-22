# !/usr/bin/python3
# encoding: utf-8

import pandas as pd
import rosbag_pandas as rbp
from datetime import datetime
import sys  

reload(sys)  
sys.setdefaultencoding('utf8')

# df1 = rbp.bag_to_dataframe('data/rosout.bag')
df1 = rbp.bag_to_dataframe('data/example2.bag')
# print(df1)
print(df1.columns.values)
print(df1.index[0])

# datetimeEpoch = datetime.strptime(df1.index[0],'%Y-%m-%d %H:%M:%S.%f')
# print(datetimeEpoch)

# print(df1['/header/stamp/secs'])
# df1.to_pickle("./data/utcTime.pkl")
# df_include = rosbag_pandas.bag_to_dataframe('data/rosout.bag', include=['/rosout']))
# print(df1)
# print( df1['/velodyne_points/data'].to_string() )

