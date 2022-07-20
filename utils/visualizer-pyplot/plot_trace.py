import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import seaborn
import numpy as np
import pandas as pd

trace_list = pd.read_csv("./trace_mass.dat", delimiter=' ', header=None).transpose().to_numpy()

fig, ax = plt.subplots()
fig.set_tight_layout(True)
ax.set(aspect=1)

ax.scatter([0.0, 39.0, 61.0, 80.0, 100.0], [0.0, 41.0, 61.0, 75.0, 100.0])
hinder_1 = plt.Circle((39.0, 41.0), 5.0, color='r')
hinder_2 = plt.Circle((61.0, 61.0), 5.0, color='r')
hinder_3 = plt.Circle((80.0, 75.0), 5.0, color='r')
ax.add_patch(hinder_1)
ax.add_patch(hinder_2)
ax.add_patch(hinder_3)

def update(i):
    label = 'timestep {0}'.format(i)
    print(label)
    # Update the line and the axes (with a new xlabel). Return a tuple of
    # "artists" that have to be redrawn for this frame.
    ax.plot(trace_list[0][:i+1], trace_list[1][:i+1])
    ax.set_xlabel(label)
    return ax
    

if __name__ == "__main__":
    anim = FuncAnimation(fig, update, frames=np.arange(0, len(trace_list[0])), interval=1) # interval=200 for normal
    try:
        anim.save('trace_mass.mp4', dpi=fig.get_dpi(), writer='imagemagick')
    except:
        try:
            anim.save('trace_mass.gif', dpi=fig.get_dpi(), writer='imagemagick')
        except:
            print("Cannot output video!\n")
    plt.show()