import numpy as np
from math import pow
from matplotlib import pyplot as plt


# Pk = np.asarray([ [0.69713919,  0.50337711,  0.62546919,  0.60379113],
#        [0.60940566,  0.54608678,  0.64439326,  0.73350994],
#        [0.52562844,  0.59335262,  0.61162154,  0.54083549],
#        [0.52965383,  0.6347104,   0.63272751,  0.64406161],
#        [0.64636854,  0.51403456,  0.69560389,  0.68897421]])

Pk = np.array([[0.72416009,  0.60500308,  0.6445418,   0.56498079],
                [0.50411137,  0.89956329,  0.52711604,  0.63562927],
                [0.59429652,  0.63194967,  0.73396595,  0.71920273],
                [0.7196956,   0.5965387,   0.54194817,  0.73771202],
                [0.67973529,  0.58949019,  0.70949019,  0.69833375]])

# Pk attrited:
# Pk = np.array([[0.72416009,  0.60500308,  0.6445418,   0.56498079],
#                 [0.50411137,  0.89956329,  0.52711604,  0.63562927],
#                 [0.59429652,  0.63194967,  0.73396595,  0.71920273],
#                 [0.7196956,   0.5965387,   0.54194817,  0.73771202],
#                 [0.67973529,  0.58949019,  0.70949019,  0.69833375]])

# Pk attrited:
# Pk = np.array([[0.5, 0.3, 0.4, 0.3],
#                [.6, .4, .4, .1],
#                [.3, .2, .6, .7],
#                [.4, .3, .7, .5]])

V = np.asarray([[6.31792338], [9.97143366],  [6.52830087],  [5.16320936]])
comp = 1 - Pk

weapons, targets = comp.shape

EV = 30.*np.ones((int(pow(targets, weapons)), 1));

for i in range(int(pow(targets, weapons))):
  achieved_comp = np.ones((targets,1));
  remainder = i;
  for j in range(weapons):
    target = int(np.floor(remainder/pow(targets, weapons-j-1)));
    remainder = np.mod(remainder, pow(targets, weapons-j-1));
    #print("W: %d \t T: %d"%(j, target))
    achieved_comp[target] = achieved_comp[target]*comp[j, target];

  value = np.dot(np.reshape(achieved_comp,-1), np.reshape(V, -1));

  EV[i] = value
  #print("EV: %f\tI: %d"%(EV[i], i))

plt.figure()
plt.plot(EV)
min_ev = np.min(EV)
i = np.argmin(EV)
plt.plot(i,min_ev,'k*', markersize=15)

print("Smallest expected value: %f @ %d"%(min_ev, i))
print("Assignment:")
remainder = i
for j in range(weapons):
  target = int(np.floor(remainder/pow(targets, weapons-j-1)))
  print("\tW: %d \t T: %d"%(j, target+1))
  remainder = np.mod(remainder, pow(targets, weapons-j-1));

sim_min = 98
plt.plot(sim_min, EV[sim_min], 'r*', markersize=15)

print("From Sim: %f @ %d"%(EV[sim_min], sim_min))
print("Assigmnent:")
remainder = sim_min;
for j in range(weapons):
  target = int(np.floor(remainder/pow(targets, weapons-j-1)))
  print("\tW: %d \t T: %d"%(j, target+1))
  remainder = np.mod(remainder, pow(targets, weapons-j-1));

plt.show()
