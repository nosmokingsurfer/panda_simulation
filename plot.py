import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("./error.csv")

plt.figure('Errors')
plt.title('End effector 3d absolte errors along exes')
plt.ylabel('m, meters')
plt.xlabel('feedback points')
plt.plot(np.abs(data[:,1]-data[:,4]), label='along x')
plt.plot(np.abs(data[:,2]-data[:,5]), label='along y')
plt.plot(np.abs(data[:,3]-data[:,6]), label='along z')
plt.legend()

plt.figure('Joints')
plt.title('Joint angles')
plt.ylabel('rad, radians')
plt.xlabel('feedback points')
plt.plot(data[:,7], label='joint[0]')
plt.plot(data[:,8], label='joint[1]')
plt.plot(data[:,9], label='joint[2]')
plt.plot(data[:,10], label='joint[3]')
plt.plot(data[:,11], label='joint[4]')
plt.plot(data[:,12], label='joint[5]')
plt.plot(data[:,13], label='joint[6]')
plt.legend()
plt.show()
