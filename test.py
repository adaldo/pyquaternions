import quaternion as qtn
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpp
import mpl_toolkits.mplot3d as mp3
import matplotlib as mpl



#rot1 = qtn.Quaternion(1,2,3,4).normalize()
#rot2 = qtn.Quaternion(2,-3,-4,-5).normalize()
rot1 = qtn.Quaternion(angle=0.0, axis=[0,0,1])
rot2 = qtn.Quaternion(angle=np.pi/4, axis=[1,1,-1])
rot3 = qtn.Quaternion(angle=np.pi/3, axis=[1,0,1])
rotations = [rot1, rot2, rot3]

TOPOLOGY = {0: set((1,2)), 1:set((0,2)), 2:set((0,1))}
COLORS = {0:"blue", 1:"red", 2:"green"}
records = [qtn.QuaternionRecord() for index in TOPOLOGY]

STEP = 5e-3
time = 0.0

mpl.rc('text', usetex=True)
plt.ion()
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("equal")
PLOT_LIM = 1.0
ax.set_xlim( [ -PLOT_LIM, PLOT_LIM ] )
ax.set_ylim( [ -PLOT_LIM, PLOT_LIM ] )
ax.set_zlim( [ -PLOT_LIM, PLOT_LIM ] )
ax.set_xlabel(r"$x$")
ax.set_ylabel(r"$y$")
ax.set_zlabel(r"$z$")





arts = [None for index in TOPOLOGY]


for index in range(1000):
    derivatives = dict()
    for index, rot in enumerate(rotations):
        ang_vel = np.zeros((3,1)).flatten()
        for nbr_idx in TOPOLOGY[index]:
            rel_rot = rotations[nbr_idx]*rot.conjugate
            if np.fabs(rel_rot.angle) > 1e-4:
                ang_vel += rel_rot.angle*rel_rot.axis
        derivatives[index] = rot.apply_ang_vel(ang_vel)
    for index, rot in enumerate(rotations):
         rot = rot + derivatives[index]*STEP
         rotations[index] = rot.normalize()
    for index, art in enumerate(arts):
        if not art is None:
            for artist in art: artist.remove()
        arts[index] = rotations[index].draw(color=COLORS[index])
    for index, rec in enumerate(records):
        rec.append(time, rotations[index])
    time += STEP
    plt.draw()

plt.ioff()
plt.figure()
for index, rec in enumerate(records):
    rec.plot(color=COLORS[index])
plt.grid(True)
#plt.ylim([-0.2,1.2])
plt.show()
