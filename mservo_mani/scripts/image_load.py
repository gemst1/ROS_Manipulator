import h5py
import matplotlib.pyplot as plt

f = h5py.File('/home/mservo/Dataset/test_0002.hdf5', 'r')
# f = h5py.File('test_0000.hdf5', 'r')
print(f['data']['joint_state'][()])
plt.figure()
plt.imshow(f['data']['image'][()])
plt.grid(False)
plt.axis('off')
plt.show()