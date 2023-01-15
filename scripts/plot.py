#/usr/bin/env python3

import h5py
import numpy as np
from matplotlib import pyplot as plt 
import glob
import os

# get latest log file
list_of_files = glob.glob('/tmp/cartesio_opensot*.mat')
latest_file = max(list_of_files, key=os.path.getctime)
print('generating plots for {} ...'.format(latest_file))

# read
mat = h5py.File(latest_file)
# for l in mat.keys(): print(l)

r_foot_errors = [k for k in mat.keys() if k.startswith('r_foot') and k.endswith('error')]
l_foot_errors = [k for k in mat.keys() if k.startswith('l_foot') and k.endswith('error')]

r_foot_ref = [k for k in mat.keys() if k.startswith('r_foot') and k.endswith('pos_ref')]
l_foot_ref = [k for k in mat.keys() if k.startswith('l_foot') and k.endswith('pos_ref')]

# r foot plot
plt.figure()
for i, v in enumerate(l_foot_ref):
    plt.subplot(2, 2, i+1)
    plt.plot(np.array(mat[v])[:, 0:3])
    plt.legend(['x', 'y', 'z'])
    plt.grid()

plt.show()
