import pickle


with open('./data/safeQuat.pkl', 'rb') as f:
    data = pickle.load(f)

print(data)