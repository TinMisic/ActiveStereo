import numpy as np
import matplotlib.pyplot as plt

# Initialize variables
positions = []
times = []
lost = None

# Open the file for reading
with open('log/lpm_sa.txt', 'r') as file:
    # Iterate over each line in the file
    for line in file:
        # Strip whitespace from the beginning and end of the line
        line = line.strip()
        # Check if the line starts with '#' indicating the last line of the file
        if line.startswith('#'):
            # Extract the integer value after 'lost' in the last line
            lost = int(line.split()[1])
        else:
            # Extract the numpy array and the time value from the line
            arr, time = line.replace('[', '').replace(']', '').split("\t",maxsplit=1)
            arr = np.array([float(x) for x in arr.split()])
            time = float(time)
            # Append the data to the list
            positions.append(arr)
            times.append(time)

# Print the parsed data and the lost value
pos = np.array(positions)/100
tru = []

# Generate true positions
A = np.array([0,10,100,1])
B = np.array([30,10,100,1])
C = np.array([-40,10,100,1])
points = [A, B, A]
bars = [7.5,55,110]

def bar(t, bars):
    for i in range(len(bars)-1):
        if t<bars[i+1]:
            return i
    return len(bars)-2
        
for t in times:
    i = bar(t, bars)
    t_start = bars[i]
    t_end = bars[i+1]

    P = (1 - ((t-t_start)/(t_end-t_start)))*points[i] + ((t-t_start)/(t_end-t_start))*points[i+1]
    tru.append(P)


tru = np.array(tru)/100
fig, axs = plt.subplots(2,2)

#x
axs[0,0].plot(times,pos[:,0],label="Estimirano",color="red")
axs[0,0].plot(times,tru[:,0], label="Stvarno",color=(200/255,200/255,0))
axs[0,0].set_title('x')
axs[0,0].set_xlabel("t (s)")
axs[0,0].set_ylabel("x koordinata (m)")
axs[0,0].set_ylim(-1,1)
axs[0,0].legend(loc=3)

#y
axs[0,1].plot(times,pos[:,1],label="Estimirano",color="green")
axs[0,1].plot(times,tru[:,1], label="Stvarno",color=(200/255,200/255,0))
axs[0,1].set_title('y')
axs[0,1].set_xlabel("t (s)")
axs[0,1].set_ylabel("y koordinata (m)")
axs[0,1].set_ylim(-1,1)
axs[0,1].legend(loc=3)

#z
axs[1,0].plot(times,pos[:,2],label="Estimirano",color="blue")
axs[1,0].plot(times,tru[:,2], label="Stvarno",color=(200/255,200/255,0))
axs[1,0].set_title('z')
axs[1,0].set_xlabel("t (s)")
axs[1,0].set_ylabel("z koordinata (m)")
axs[1,0].set_ylim(-1,3)
axs[1,0].legend(loc=3)

#odstupanje
mse = np.mean((tru - pos)[:,:3]**2)
print("MSE:",mse)
axs[1,1].plot(times, np.linalg.norm((tru - pos)[:,:3],axis=1),color="purple")
axs[1,1].set_xlabel("t (s)")
axs[1,1].set_ylabel("odstupanje (m)")
axs[1,1].set_title("Odstupanje, MSE="+"{:.3f}".format(mse))
# ax.set_ylim([-40,40])

# fig.suptitle("Kartezijska slika bez smetnje")
plt.tight_layout()
plt.show()
print('Lost value:', lost)
print(np.min(tru[:,0]))
