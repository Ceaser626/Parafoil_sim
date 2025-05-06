import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import art3d


def plot_traj():
    # Load data
    states = pd.read_csv('cmake-build-debug\simulation_states.csv')

    # Plot
    cm = plt.get_cmap('tab10')
    fig, ax = plt.subplots(subplot_kw={'projection': '3d'})
    y = states['y']
    ax.plot3D(states['y'], states['x'], states['z'], color=cm.colors[3], linewidth=1.5, label='delta_l = 0, delta_r = 0.3')
    ax.grid(linestyle='--')
    legend = plt.legend(loc='best', fancybox=True, edgecolor='black', framealpha=1)
    ax.zaxis.set_rotate_label(False)
    ax.set_xlabel('Y-axis, m', labelpad=6)
    ax.set_ylabel('X-axis, m', labelpad=6)
    ax.set_zlabel('Altitude, m', rotation=90, labelpad=6)
    ax.zaxis._axinfo['juggled'] = (1, 2, 0)
    limits = np.array([getattr(ax, f'get_{axis}lim')() for axis in 'xyz'])
    ax.set_box_aspect(np.ptp(limits, axis=1))
    xlims, ylims, zlims = lims(ax.get_xlim()), lims(ax.get_ylim()), lims(ax.get_zlim())
    i = np.array([xlims[0], ylims[0], zlims[0]])
    f = np.array([xlims[0], ylims[0], zlims[1]])
    p = art3d.Poly3DCollection(np.array([[i, f]]))
    p.set_color('black')
    ax.add_collection3d(p)
    ax.xaxis.pane.set_edgecolor('#000000')
    ax.yaxis.pane.set_edgecolor('#000000')
    ax.zaxis.pane.set_edgecolor('#000000')
    ax.xaxis.pane.set_alpha(1)
    ax.yaxis.pane.set_alpha(1)
    ax.zaxis.pane.set_alpha(1)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.view_init(elev=33, azim=-47)
    fig.set_size_inches(6.4, 6)
    fig.set_tight_layout(True)
    plt.savefig(f'figure/Fig1.tif', dpi=300)
    plt.savefig(f'figure/Fig1.pdf', dpi=300)
    plt.show()


def lims(mplotlims):
    scale = 1.021
    offset = (mplotlims[1] - mplotlims[0]) * scale
    return mplotlims[1] - offset, mplotlims[0] + offset


if __name__ == '__main__':
    plot_traj()
