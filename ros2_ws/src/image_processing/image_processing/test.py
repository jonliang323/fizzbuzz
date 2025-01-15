import numpy as np

a = np.array([[1,2,3,3,3,4],[1,2,3,3,3,4],[3,3,3,1,1,4]])
a[:1] = 0
print(a)