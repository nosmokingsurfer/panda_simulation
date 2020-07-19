import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("./error.txt")


plt.title('End effector 3d absolte errors along exes')
plt.ylabel('m, meters')
plt.xlabel('feedback points')
plt.plot(np.abs(data[:,1]-data[:,4]), label='along x')
plt.plot(np.abs(data[:,2]-data[:,5]), label='along y')
plt.plot(np.abs(data[:,3]-data[:,6]), label='along z')
plt.legend()
plt.show()
