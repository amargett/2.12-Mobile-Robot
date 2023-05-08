import numpy as np

a = 360 - (np.degrees(np.arctan2(1, 1)) + 360) % 360
b = 360 - (np.degrees(np.arctan2(1, -1))  + 360) % 360
c = 360 - (np.degrees(np.arctan2(-1, -1))  + 360) % 360
d = 360 - (np.degrees(np.arctan2(-1, 1))  + 360) % 360

print(a, b, c, d)