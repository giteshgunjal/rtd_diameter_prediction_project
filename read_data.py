import pickle
import numpy as np
import matplotlib.pyplot as plt


# open a file, where you stored the pickled data
file = open('velocity-risk-ego-rover-7-overtake_1.dat', 'rb')

# dump information to that file
data = pickle.load(file)

# close the file

file.close()

print('Showing the pickled data:')

data = np.asarray(data)
data[:,0] = data[:,0] - data[0,0]
print(data.shape)
plt.plot(data[:,0],data[:,1])
plt.plot(data[:,0],data[:,2])
plt.show()
