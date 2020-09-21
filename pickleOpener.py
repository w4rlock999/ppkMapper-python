import pickle


# with open('./data/TestFlightRejodaniSore/map/pkl/utcData.pkl', 'rb') as f:
#     data = pickle.load(f)
index = 4
timeConverterFilePath = './data/TestFlightRejodaniSore/map/pkl/imuData.pkl'

with open(timeConverterFilePath, 'rb') as f:
    timeData_raw = pickle.load(f, encoding="latin1")
print("epoch bag time")
print(float(timeData_raw.index[index]))
print("epoch message time")
print(timeData_raw['/ppk_quat/header/stamp/secs'][timeData_raw.index[index]])
print(timeData_raw['/ppk_quat/header/stamp/nsecs'][timeData_raw.index[index]])