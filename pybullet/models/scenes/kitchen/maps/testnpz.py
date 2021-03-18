import numpy as np
"""
data = np.load('maps.npz')
lst = data.files
for item in lst:
    print(item)
    print((data[item]))

x= data['drivable_area']
print('drivable_area',(np.where(x !=0)[0]).shape)

x= data['norm_distance_from_start']
print('norm_distance_from_start',(np.where(x !=0)[0]).shape)

x= data['norm_distance_to']
print('norm_distance_to',(np.where(x !=0)[0]).shape)

x= data['norm_distance_to_obstacle']
print('norm_distance_to_obstacle',(np.where(x !=0)[0]).shape)
"""
filename = "maps.npz"
filedic = dict(np.load(filename))
print(filedic)
del filedic['properties']
del filedic['norm_distance_to']
del filedic['norm_distance_to_obstacle']



#print(filedic['norm_distance_from_start'])

np.savez(filename, **filedic)

#filedic['myentry'] = mynewvalue
#np.savez(filename, **filedic)
