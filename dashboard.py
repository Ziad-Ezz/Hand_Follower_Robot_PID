import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation

AXIS_NAMES = ["X (Left/Right)", "Y (Depth/Forward)", "Z (Height/Up)"]

def run_dashboard(state):
    fig = plt.figure(figsize=(14, 8), facecolor="#0d1117")
    fig.canvas.manager.set_window_title("Tustin PID Robotic Arm Dashboard")

    gs = gridspec.GridSpec(3, 2, figure=fig, left=0.05, right=0.95, top=0.90, bottom=0.10, 
                           wspace=0.25, hspace=0.55, width_ratios=[1.2, 1])

    ax3d = fig.add_subplot(gs[:, 0], projection='3d')
    ax3d.set_facecolor("#0d1117")
    ax3d.set_xlim(-0.9, 0.9); ax3d.set_ylim(-0.9, 0.9); ax3d.set_zlim(0, 1.0)
    ax3d.tick_params(colors='grey', labelsize=7)
    for pane in [ax3d.xaxis.pane, ax3d.yaxis.pane, ax3d.zaxis.pane]:
        pane.fill = False; pane.set_edgecolor('#222')
    
    ax3d.set_title("3D Live View: Tustin PID (Cyan)", color="white", pad=8)

    arm_line,  = ax3d.plot([], [], [], 'o-', color='#00e5ff', lw=4, ms=6, label='Robot Arm')
    target_dot,= ax3d.plot([], [], [], 'o', color='#ff4444', ms=10, label='Hand Target')
    end_eff_dot,= ax3d.plot([], [], [], '*', color='#00e5ff', ms=14, label='End Effector')
    conn_line, = ax3d.plot([], [], [], ':', color='#ffffff', lw=2)
    ax3d.legend(loc='upper right', fontsize=8, facecolor='#111', edgecolor='#333', labelcolor='white')

    row_axes = []
    lines = {}
    for row in range(3):
        ax = fig.add_subplot(gs[row, 1])
        ax.set_facecolor("#0d1117")
        ax.tick_params(colors='grey', labelsize=7)
        ax.set_title(f"{AXIS_NAMES[row]} Tracking", color="white", fontsize=9)
        for sp in ['top', 'right', 'bottom', 'left']: ax.spines[sp].set_color('#222')

        l_sp,   = ax.plot([], [], color='#ff4444', lw=1.5, label='Target')
        l_pid,  = ax.plot([], [], color='#00e5ff', lw=2.0, label='PID Output')
        ax.legend(fontsize=7, facecolor='#111', edgecolor='#333', labelcolor='white', loc='upper left')

        row_axes.append(ax)
        lines[row] = (l_sp, l_pid)

    def update(_):
        with state.lock:
            joints = [j.copy() for j in state.joints]
            hp = state.smoothed_target.copy()
            curr_pos = state.current_position.copy()
            
            t      = list(state.timestamps)
            sps    = [list(q) for q in state.targets]
            p_meas = [list(q) for q in state.positions]

        if joints and len(joints) > 0:
            arm_line.set_data_3d([j[0] for j in joints], [j[1] for j in joints], [j[2] for j in joints])
            
        target_dot.set_data_3d([hp[0]], [hp[1]], [hp[2]])
        end_eff_dot.set_data_3d([curr_pos[0]], [curr_pos[1]], [curr_pos[2]])
        conn_line.set_data_3d([curr_pos[0], hp[0]], [curr_pos[1], hp[1]], [curr_pos[2], hp[2]])

        if len(t) > 2:
            for i in range(3):
                l_sp, l_pid = lines[i]
                ax = row_axes[i]

                l_sp.set_data(t, sps[i])
                l_pid.set_data(t, p_meas[i])

                ax.set_xlim(t[0], t[-1] + 0.1)
                all_vals = sps[i] + p_meas[i]
                if all_vals:
                    mn, mx = min(all_vals), max(all_vals)
                    pad = max(abs(mx - mn) * 0.15, 0.05)
                    ax.set_ylim(mn - pad, mx + pad)

        return (arm_line, target_dot, end_eff_dot, conn_line, *[l for ls in lines.values() for l in ls])

    ani = FuncAnimation(fig, update, interval=80, blit=False, cache_frame_data=False)
    plt.show()
    state.running = False
    return ani