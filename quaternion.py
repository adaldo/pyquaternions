import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpp
import mpl_toolkits.mplot3d as mp3
import matplotlib as mpl



class Arrow3D(mpp.FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
        mpp.FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = mp3.proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        mpp.FancyArrowPatch.draw(self, renderer)




class Quaternion:

    def __init__(self, *args, **kwargs):
        if len(args) is 4:
            vec = np.array(args).flatten()
            self.__init__(vec)
            return
        elif len(args) is 1:
            vec = np.array(args)
            self._data = vec.flatten()
            return
        elif len(kwargs) is 2:
            if "scalar" in kwargs and "vector" in kwargs:
                self.__init__(kwargs["scalar"], *(kwargs["vector"].tolist()))
                return
            if "axis" in kwargs and "angle" in kwargs:
                if kwargs["angle"] > np.pi or kwargs["angle"] < -np.pi:
                    raise ValueError()
                self.__init__(scalar=np.cos(kwargs["angle"]/2), vector=np.sin(kwargs["angle"]/2)*np.array(kwargs["axis"]))
                return
        raise TypeError()

    def __str__(self):
        return str(self._data)

    @property
    def xvector(self):
        return self*np.array([1,0,0]).flatten()

    @property
    def yvector(self):
        return self*np.array([0,1,0]).flatten()

    @property
    def zvector(self):
        return self*np.array([0,0,1]).flatten()

    def draw(self, offset=None, **kwargs):
        if offset is None:
            offset = (0.,0.,0.)
        x0, y0, z0 = offset
        arrows = list()
        vec = self.xvector
        xarrow = Arrow3D([x0, x0+vec[0]], [y0, y0+vec[1]], [z0, z0+vec[2]], arrowstyle="-|>", mutation_scale=20, **kwargs)
        vec = self.yvector
        yarrow = Arrow3D([x0, x0+vec[0]], [y0, y0+vec[1]], [z0, z0+vec[2]], arrowstyle="-|>", mutation_scale=20, linestyle="dashed", **kwargs)
        vec = self.zvector
        zarrow = Arrow3D([x0, x0+vec[0]], [y0, y0+vec[1]], [z0, z0+vec[2]], arrowstyle="-|>", mutation_scale=20, linestyle="dotted", **kwargs)
        for arrow in (xarrow, yarrow, zarrow):
            plt.gca(projection="3d").add_artist(arrow)
        return xarrow, yarrow, zarrow

    @property
    def angle(self):
        return 2*np.arccos(self.scalar)

    @property
    def axis(self):
        return self.vector/np.sqrt(1-self.scalar**2)

    @property
    def scalar(self):
        return self._data[0]

    @property
    def vector(self):
        return np.array(self._data[1:])

    @property
    def conjugate(self):
        return Quaternion(scalar=self.scalar, vector=-self.vector)

    @property
    def inverse(self):
        return self.conjugate/np.linalg.norm(self._data)**2

    def normalize(self):
        return Quaternion(self._data/np.linalg.norm(self._data))

    def __neg__(self):
        return Quaternion(-self._data)

    def __add__(self, other):
        return Quaternion(self._data+other._data)

    def __sub__(self, other):
        return self + (-other)

    def __mul__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(other*self._data)
        if isinstance(other, np.ndarray):
            w, qv = self.scalar, self.vector
            return 2.0*(qv.dot(other))*qv + (w*w-qv.dot(qv))*other + 2.0*w*np.cross(qv, other)
        elif isinstance(other, Quaternion):
            scalar = self.scalar*other.scalar - self.vector.dot(other.vector)
            vector = self.scalar*other.vector + other.scalar*self.vector + np.cross(self.vector, other.vector)
            return Quaternion(scalar=scalar, vector=vector)
        raise TypeError()

    def __rmul__(self, other):
        if isinstance(other, (int, float)):
            return Quaternion(other*self._data)
        raise TypeError()

    def __div__(self, other):
        if isinstance(other, Quaternion):
            return self*other.inverse
        if isinstance(other, (int, float)):
            return Quaternion(self._data/other)
        raise TypeError()

    def apply_ang_vel(self, ang_vel):
        ang_vel_quaternion = Quaternion(scalar=0.0, vector=ang_vel)
        return 0.5*ang_vel_quaternion*self

    def distance_squared(self, other):
        rel = self/other
        return rel.angle**2




class QuaternionRecord:

    def __init__(self):
        self._trecord = list()
        self._nrecord = list()
        self._xrecord = list()
        self._yrecord = list()
        self._zrecord = list()

    def append(self, time, quaternion):
        self._trecord.append(time)
        self._nrecord.append(quaternion.scalar)
        vec = quaternion.vector
        self._xrecord.append(vec[0])
        self._yrecord.append(vec[1])
        self._zrecord.append(vec[2])

    def plot(self, *args, **kwargs):
        plt.plot(self._trecord, self._nrecord, linestyle="-.", label=r"$n$", *args, **kwargs)
        plt.plot(self._trecord, self._xrecord, linestyle="solid", label=r"$x$", *args, **kwargs)
        plt.plot(self._trecord, self._yrecord, linestyle="dashed", label=r"$y$", *args, **kwargs)
        plt.plot(self._trecord, self._zrecord, linestyle="dotted", label=r"$z$", *args, **kwargs)
