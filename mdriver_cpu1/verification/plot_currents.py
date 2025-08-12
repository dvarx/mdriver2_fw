import matplotlib.pyplot as plt
import numpy

currents=[]
with open("/home/dvarx/src/mdriver2_fw/mdriver_cpu1/verification/current_buffer.dat","r") as fptr:
    fptr.readline()
    for line in fptr:
        currents.append(float(line))

plt.plot(currents)
plt.ylabel("Sensor measurement")
plt.show()