# dynamics_sim.py
# -----------------------------------------------------------------------------
# Rigid Body Dynamics Simulation for Cam-Wheel System
# -----------------------------------------------------------------------------
# Modified to include Floor/Ceiling constraints AND real-time Y-plot in GIF.

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter
import initial_state_generator_gemini as ic_generator
import sys

# ==========================================
# 1. MATERIAL & SURFACE DATABASE
# ==========================================

MATERIALS = {
    'Steel': {'zeta': 0.02, 'name': 'Steel'},
    'Aluminum': {'zeta': 0.05, 'name': 'Aluminum'},
    'Nylon': {'zeta': 0.30, 'name': 'Nylon'},
    'Rubber': {'zeta': 0.90, 'name': 'Rubber'},
}

SURFACES = {
    'Dry': 0.35,
    'Greased': 0.10,
    'Oiled': 0.05,
    'Rough': 0.60,
}


# ==========================================
# 2. PARAMETERS
# ==========================================

class SimParams:
    def __init__(self):
        # --- Visualization & Output ---
        self.animate = True
        self.save_gif = True
        self.show_window = True

        self.gif_real_time_length = 8.0  # [s] Target duration
        self.gif_fps = 30  # [fps]

        # --- Time ---
        self.dt = 2.5e-6
        self.t_end = 0.1

        # --- Geometry ---
        self.geo = {
            "N": 7,
            "d": 0.001,
            "R_s": 0.002,
            "R": 0.008,
            "socket_angle_deg": 5,
            "theta0": np.pi / 2
        }

        # --- Mass Properties ---
        self.I_axis_factor = 0.1

        # --- Input Force ---
        self.F_input_max = 8
        self.t_input_ramp = 0.1

        # --- Vertical Spring & Limits ---
        self.k_vert = 4100.0  # [N/m]
        self.F_preload = 4.6  # [N]

        # *** Vertical Limits ***
        self.max_disp_y = 0.001  # [m] 1mm ceiling limit

        # --- Horizontal Latch ---
        self.L_latch = 0.006
        self.F_latch_release = 0.0

        # --- Torsion Spring ---
        self.k_tor = 0.18
        self.c_tor = 0.001

        # --- Smart Material Selection ---
        self.mat_type = 'Steel'
        self.surf_type = 'Greased'

        self.mu_friction = SURFACES[self.surf_type]

        # --- Safety Limits ---
        self.max_vel_rot = 5000.0
        self.max_force = 1000.0

        # --- Derived Parameters ---
        total_phys_steps = self.t_end / self.dt
        total_frames = self.gif_real_time_length * self.gif_fps
        self.anim_steps = int(total_phys_steps / total_frames)
        if self.anim_steps < 1: self.anim_steps = 1


# ==========================================
# 3. MATH HELPERS
# ==========================================

def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def get_closest_point_on_segment(P, A, B):
    AP = P - A
    AB = B - A
    ab2 = np.dot(AB, AB)
    if ab2 < 1e-12: return A
    t = np.dot(AP, AB) / ab2
    t = np.clip(t, 0.0, 1.0)
    return A + t * AB


# ==========================================
# 4. DYNAMICS ENGINE
# ==========================================

class CamSystemSimulation:
    def __init__(self, params):
        self.p = params

        # --- Initialize State ---
        ic = ic_generator.get_initial_state_from_params(**self.p.geo)
        self.C0 = np.array(ic['C0'])
        self.segs_local = ic['geom']['segs_w']

        # --- Transform segments to local frame ---
        R_inv = rot2(-self.p.geo['theta0'])

        pts = []
        for s in self.segs_local:
            s['p0'] = R_inv @ s['p0']
            s['p1'] = R_inv @ s['p1']
            pts.append(s['p0'])
        if len(self.segs_local) > 0:
            pts.append(self.segs_local[0]['p0'])
        self.wheel_pts_local = np.array(pts)

        self.m_c = ic['mass_props']['circle']['m']
        self.I_w = ic['mass_props']['wheel']['I']
        self.I_a = self.I_w * self.p.I_axis_factor

        self.tune_contact_physics()
        self.check_rotational_stability()

        self.state = {
            'c_pos': self.C0.copy(),
            'c_vel': np.zeros(2),
            'w_theta': self.p.geo['theta0'],
            'w_omega': 0.0,
            'a_theta': self.p.geo['theta0'],
            'a_omega': 0.0
        }

        self.is_latched = False
        self.history = {k: [] for k in
                        ['t', 'c_x', 'c_y', 'w_theta', 'w_omega', 'a_theta', 'a_omega', 'f_contact', 'torque_spring']}

        # Visualization Objects
        self.fig_anim = None
        self.ax_geo = None
        self.ax_disp = None
        self.ax_disp_twin = None  # NEW: Twin axis for Y displacement
        self.ax_force = None
        self.ax_force_twin = None

        self.ln_wheel = None
        self.ln_axis = None
        self.patch_circle = None
        self.txt_info = None

        self.pt_disp_x = None
        self.pt_disp_y = None  # NEW: Marker for Y displacement
        self.pt_torque = None
        self.pt_force = None

    def check_rotational_stability(self):
        print("-" * 50)
        print("ROTATIONAL STABILITY CHECK")
        if self.I_w > 0:
            wn_tor = np.sqrt(self.p.k_tor / self.I_w)
            freq_hz = wn_tor / (2 * np.pi)
            print(f"Torsion Nat. Freq: {wn_tor:.0f} rad/s ({freq_hz:.0f} Hz)")
        print("-" * 50)

    def tune_contact_physics(self):
        safety_factor = 0.25
        k_max_stable = safety_factor * 4 * self.m_c / (self.p.dt ** 2)
        self.k_contact = k_max_stable
        mat_props = MATERIALS[self.p.mat_type]
        zeta = mat_props['zeta']
        self.c_contact = 2 * zeta * np.sqrt(self.k_contact * self.m_c)

    def get_input_force(self, t):
        if t < self.p.t_input_ramp:
            return (self.p.F_input_max / self.p.t_input_ramp) * t
        else:
            return self.p.F_input_max

    def detect_contact(self):
        R_mat_T = rot2(-self.state['w_theta'])
        C_local = R_mat_T @ self.state['c_pos']

        max_pen = -1e9
        best_contact = None

        for seg in self.segs_local:
            P0 = seg['p0']
            P1 = seg['p1']
            Q_local = get_closest_point_on_segment(C_local, P0, P1)
            dist = np.linalg.norm(C_local - Q_local)
            pen = self.p.geo['R_s'] - dist

            if pen > 0 and pen > max_pen:
                max_pen = pen
                if dist < 1e-12:
                    d = P1 - P0
                    n_local = np.array([-d[1], d[0]])
                    n_local /= np.linalg.norm(n_local)
                else:
                    n_local = (C_local - Q_local) / dist

                R_mat = rot2(self.state['w_theta'])
                n_world = R_mat @ n_local
                Q_world = R_mat @ Q_local
                best_contact = (pen, n_world, Q_world)
        return best_contact

    def step(self, t):
        s = self.state
        p = self.p

        F_c_net = np.zeros(2)
        tau_w_net = 0.0
        tau_a_net = 0.0

        F_in = self.get_input_force(t)
        F_c_net[0] += F_in

        # Vertical Spring
        dy = s['c_pos'][1] - self.C0[1]
        F_vert = -p.k_vert * dy - p.F_preload
        F_c_net[1] += F_vert

        d_theta = s['w_theta'] - s['a_theta']
        d_omega = s['w_omega'] - s['a_omega']
        tau_spring = -p.k_tor * d_theta - p.c_tor * d_omega

        tau_w_net += tau_spring
        tau_a_net -= tau_spring

        contact = self.detect_contact()
        f_contact_mag = 0.0

        if contact:
            pen, n_vec, Q_world = contact
            r_w = Q_world
            v_w_point = np.array([-s['w_omega'] * r_w[1], s['w_omega'] * r_w[0]])
            v_c_point = s['c_vel']

            v_rel = v_c_point - v_w_point
            v_rel_n = np.dot(v_rel, n_vec)

            Fn = max(0.0, self.k_contact * (pen ** 1.5) - self.c_contact * v_rel_n)
            Fn = min(Fn, p.max_force)
            f_contact_mag = Fn
            F_N_vec = Fn * n_vec
            F_c_net += F_N_vec

            tau_contact = r_w[0] * (-F_N_vec[1]) - r_w[1] * (-F_N_vec[0])
            tau_w_net += tau_contact

            t_vec = np.array([-n_vec[1], n_vec[0]])
            v_rel_t = np.dot(v_rel, t_vec)
            F_f_mag = p.mu_friction * Fn
            F_F_vec = -np.sign(v_rel_t) * F_f_mag * t_vec

            F_c_net += F_F_vec
            tau_friction = r_w[0] * (-F_F_vec[1]) - r_w[1] * (-F_F_vec[0])
            tau_w_net += tau_friction

        # --- CONSTRAINTS ---

        # 1. Left Wall (Start Limit)
        if s['c_pos'][0] < self.C0[0]:
            s['c_pos'][0] = self.C0[0]
            if s['c_vel'][0] < 0:
                s['c_vel'][0] = 0.0
                if F_c_net[0] < 0: F_c_net[0] = 0.0

        # 2. Vertical Constraints (Floor & Ceiling)
        current_y = s['c_pos'][1]
        floor_y = self.C0[1]
        ceiling_y = self.C0[1] + p.max_disp_y

        if current_y < floor_y:
            s['c_pos'][1] = floor_y
            if s['c_vel'][1] < 0: s['c_vel'][1] = 0.0
        elif current_y > ceiling_y:
            s['c_pos'][1] = ceiling_y
            if s['c_vel'][1] > 0: s['c_vel'][1] = 0.0

        # 3. Latch Logic
        dx = s['c_pos'][0] - self.C0[0]
        if not self.is_latched:
            if dx >= p.L_latch:
                self.is_latched = True
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0.0
                F_c_net[0] = 0.0

        if self.is_latched:
            if F_c_net[0] < -p.F_latch_release:
                self.is_latched = False
            else:
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0.0
                F_c_net[0] = 0.0

        # --- INTEGRATION ---
        alpha_a = tau_a_net / self.I_a
        if s['a_omega'] <= 1e-5 and tau_a_net < 0:
            alpha_a = 0.0
            s['a_omega'] = 0.0

        acc_c = F_c_net / self.m_c
        alpha_w = tau_w_net / self.I_w

        s['c_vel'] += acc_c * p.dt
        s['w_omega'] += alpha_w * p.dt
        s['a_omega'] += alpha_a * p.dt

        s['w_omega'] = np.clip(s['w_omega'], -p.max_vel_rot, p.max_vel_rot)
        s['a_omega'] = np.clip(s['a_omega'], -p.max_vel_rot, p.max_vel_rot)

        if self.is_latched: s['c_vel'][0] = 0.0
        if s['c_pos'][0] < self.C0[0]: s['c_pos'][0] = self.C0[0]

        s['c_pos'] += s['c_vel'] * p.dt
        s['w_theta'] += s['w_omega'] * p.dt
        s['a_theta'] += s['a_omega'] * p.dt

        self.history['t'].append(t)
        self.history['c_x'].append(s['c_pos'][0])
        self.history['c_y'].append(s['c_pos'][1])
        self.history['w_theta'].append(s['w_theta'])
        self.history['w_omega'].append(s['w_omega'])
        self.history['a_theta'].append(s['a_theta'])
        self.history['a_omega'].append(s['a_omega'])
        self.history['f_contact'].append(f_contact_mag)
        self.history['torque_spring'].append(tau_spring)

    def draw_progress_bar(self, current, total, bar_length=30):
        percent = float(current) * 100 / total
        arrow = '-' * int(percent / 100 * bar_length - 1) + '>'
        spaces = ' ' * (bar_length - len(arrow))
        sys.stdout.write(f"\rProgress: [{arrow + spaces}] {percent:.1f}%")
        sys.stdout.flush()

    def run(self):
        steps = int(self.p.t_end / self.p.dt)
        print(f"Starting Simulation: {steps} steps...")

        t = 0.0
        for i in range(steps):
            self.step(t)
            if i % 2000 == 0: self.draw_progress_bar(i, steps)
            t += self.p.dt
        self.draw_progress_bar(steps, steps)
        print("\nPhysics Complete.")

        if self.p.animate:
            self.generate_animation(steps)

        return self.history

    def init_anim_static(self):
        if not self.p.save_gif and self.p.show_window:
            plt.ion()

        self.fig_anim = plt.figure(figsize=(16, 8))
        gs = self.fig_anim.add_gridspec(2, 2, width_ratios=[1.5, 1.0])

        # --- 1. Geometry (Left) ---
        self.ax_geo = self.fig_anim.add_subplot(gs[:, 0])
        self.ax_geo.set_aspect('equal')
        self.ax_geo.grid(True)
        self.ax_geo.set_title("System Animation")
        self.ax_geo.set_xlabel("X [m]")
        self.ax_geo.set_ylabel("Y [m]")

        R = self.p.geo['R']
        L = self.p.L_latch
        xlim_max = max(1.5 * R, L * 1.2)
        self.ax_geo.set_xlim(-1.5 * R, xlim_max)
        self.ax_geo.set_ylim(-1.5 * R, 1.5 * R)

        # Boundaries
        floor_y = self.C0[1]
        ceiling_y = self.C0[1] + self.p.max_disp_y
        self.ax_geo.axhline(y=floor_y, color='blue', linestyle=':', alpha=0.5, label='Min Height (Floor)')
        self.ax_geo.axhline(y=ceiling_y, color='red', linestyle=':', alpha=0.5, label='Max Height')

        self.ln_wheel, = self.ax_geo.plot([], [], 'k-', lw=1.5, label='Cam Wheel')
        self.ln_axis, = self.ax_geo.plot([], [], 'g--', lw=2, label='Axis')
        self.ln_wall_left = self.ax_geo.axvline(x=self.C0[0], color='gray', linestyle=':', lw=1, label='Start Limit')
        latch_x = self.C0[0] + self.p.L_latch
        self.ln_wall_right = self.ax_geo.axvline(x=latch_x, color='red', linestyle='--', lw=2, label='Latch')
        self.patch_circle = Circle((0, 0), radius=self.p.geo['R_s'], color='red', alpha=0.6, label='Driver')
        self.ax_geo.add_patch(self.patch_circle)
        self.ax_geo.legend(loc='lower right')
        self.txt_info = self.ax_geo.text(0.02, 0.95, '', transform=self.ax_geo.transAxes)

        # --- PREPARE DATA ---
        ts = np.array(self.history['t'])
        xs = np.array(self.history['c_x']) * 1000  # mm
        y_disp = (np.array(self.history['c_y']) - self.C0[1]) * 1000  # mm
        torques = np.array(self.history['torque_spring'])
        forces = np.array(self.history['f_contact'])

        # --- 2. Graph: Displacement X & Y (Top Right) ---
        self.ax_disp = self.fig_anim.add_subplot(gs[0, 1])
        self.ax_disp_twin = self.ax_disp.twinx()  # Twin Axis for Y

        self.ax_disp.grid(True)
        self.ax_disp.set_title("Circle Displacement")
        self.ax_disp.set_xlabel("Time [s]")
        self.ax_disp.set_ylabel("Horiz. Disp X [mm]", color='red')
        self.ax_disp_twin.set_ylabel("Vert. Disp Y [mm]", color='cyan')

        self.ax_disp.set_xlim(0, self.p.t_end)

        # Plot Lines
        self.ax_disp.plot(ts, xs, 'r-', lw=1.5, alpha=0.5)
        self.ax_disp_twin.plot(ts, y_disp, color='cyan', linestyle='-', lw=1.5, alpha=0.6)

        # Plot Markers
        self.pt_disp_x, = self.ax_disp.plot([], [], 'ro', ms=6)
        self.pt_disp_y, = self.ax_disp_twin.plot([], [], 'o', color='cyan', ms=6)

        self.ax_disp.tick_params(axis='y', labelcolor='red')
        self.ax_disp_twin.tick_params(axis='y', labelcolor='cyan')

        # --- 3. Graph: Forces (Bottom Right) ---
        self.ax_force = self.fig_anim.add_subplot(gs[1, 1], sharex=self.ax_disp)
        self.ax_force_twin = self.ax_force.twinx()
        self.ax_force.grid(True)
        self.ax_force.set_title("Dynamic Loads")
        self.ax_force.set_xlabel("Time [s]")
        self.ax_force.set_ylabel("Spring Torque [Nm]", color='purple')
        self.ax_force_twin.set_ylabel("Contact Force [N]", color='orange')

        self.ax_force.plot(ts, torques, color='purple', lw=1.5, alpha=0.5)
        self.pt_torque, = self.ax_force.plot([], [], 'o', color='purple', ms=6)

        self.ax_force_twin.plot(ts, forces, color='orange', alpha=0.5, lw=1.5)
        self.pt_force, = self.ax_force_twin.plot([], [], 'o', color='orange', ms=6)

        self.ax_force.tick_params(axis='y', labelcolor='purple')
        self.ax_force_twin.tick_params(axis='y', labelcolor='orange')

        plt.tight_layout()

    def update_anim_frame(self, frame_idx):
        idx = frame_idx * self.p.anim_steps
        if idx >= len(self.history['t']): idx = len(self.history['t']) - 1

        w_theta = self.history['w_theta'][idx]
        a_theta = self.history['a_theta'][idx]
        c_pos = [self.history['c_x'][idx], self.history['c_y'][idx]]
        t_curr = self.history['t'][idx]
        is_latched = (abs(c_pos[0] - (self.C0[0] + self.p.L_latch)) < 1e-5)

        # Update Geometry
        R_mat = rot2(w_theta)
        pts_world = (R_mat @ self.wheel_pts_local.T).T
        self.ln_wheel.set_data(pts_world[:, 0], pts_world[:, 1])

        R_axis = rot2(a_theta)
        axis_len = self.p.geo['R'] * 0.5
        p_ax = np.array([[0, 0], [axis_len, 0]])
        p_ax_w = (R_axis @ p_ax.T).T
        self.ln_axis.set_data(p_ax_w[:, 0], p_ax_w[:, 1])

        self.patch_circle.center = c_pos
        state_str = "LATCHED" if is_latched else "FREE"
        self.txt_info.set_text(f"t={t_curr:.4f}s | {state_str}")

        # Update Displacement Graph (X and Y)
        x_val = self.history['c_x'][idx] * 1000
        y_val = (self.history['c_y'][idx] - self.C0[1]) * 1000

        self.pt_disp_x.set_data([t_curr], [x_val])
        self.pt_disp_y.set_data([t_curr], [y_val])

        # Update Forces Graph
        torq_val = self.history['torque_spring'][idx]
        force_val = self.history['f_contact'][idx]
        self.pt_torque.set_data([t_curr], [torq_val])
        self.pt_force.set_data([t_curr], [force_val])

        return self.ln_wheel, self.ln_axis, self.patch_circle

    def generate_animation(self, total_phys_steps):
        total_frames = int(total_phys_steps / self.p.anim_steps)
        if total_frames < 1: total_frames = 1

        print(f"Generating Animation: {total_frames} frames...")
        self.init_anim_static()

        if self.p.save_gif:
            print("Saving GIF to disk...")
            ani = FuncAnimation(
                self.fig_anim,
                self.update_anim_frame,
                frames=total_frames,
                blit=False
            )
            gif_filename = "cam_system_dynamics.gif"
            ani.save(gif_filename, writer=PillowWriter(fps=self.p.gif_fps))
            print(f"Done! Saved to {gif_filename}")
            plt.close(self.fig_anim)
        elif self.p.show_window:
            print("Showing Animation Window...")
            for i in range(total_frames):
                self.update_anim_frame(i)
                self.fig_anim.canvas.draw()
                self.fig_anim.canvas.flush_events()
            plt.ioff()
            plt.close(self.fig_anim)


# ==========================================
# 5. PLOTTING (Static Final)
# ==========================================

def plot_results(hist):
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Subplot 1: Angles
    axs[0].plot(hist['t'], np.degrees(hist['w_theta']), label='Wheel Angle', color='blue')
    axs[0].plot(hist['t'], np.degrees(hist['a_theta']), label='Axis Angle', color='green', linestyle='--')
    axs[0].set_ylabel('Angle [deg]')
    axs[0].legend(loc='upper left')
    axs[0].grid(True)
    axs[0].set_title('Rotational Dynamics')

    # Subplot 2: Displacements
    ln1 = axs[1].plot(hist['t'], np.array(hist['c_x']) * 1000, label='Horiz. Disp. (X)', color='red')
    axs[1].set_ylabel('Horiz. Disp X [mm]', color='red')
    axs[1].tick_params(axis='y', labelcolor='red')
    axs[1].grid(True)
    axs[1].set_title('Circle Motion (X & Y)')

    ax1b = axs[1].twinx()
    y_start = hist['c_y'][0]
    y_disp = (np.array(hist['c_y']) - y_start) * 1000
    ln2 = ax1b.plot(hist['t'], y_disp, label='Vert. Disp. (Y)', color='cyan', linestyle='-')
    ax1b.set_ylabel('Vert. Disp Y [mm]', color='cyan')
    ax1b.tick_params(axis='y', labelcolor='cyan')

    lns = ln1 + ln2
    labs = [l.get_label() for l in lns]
    axs[1].legend(lns, labs, loc='upper left')

    # Subplot 3: Forces
    ax3 = axs[2]
    ln3 = ax3.plot(hist['t'], hist['torque_spring'], label='Spring Torque [Nm]', color='purple')
    ax3.set_ylabel('Torque [Nm]', color='purple')
    ax3.tick_params(axis='y', labelcolor='purple')

    ax3b = ax3.twinx()
    ln4 = ax3b.plot(hist['t'], hist['f_contact'], label='Contact Force [N]', color='orange', alpha=0.6)
    ax3b.set_ylabel('Force [N]', color='orange')
    ax3b.tick_params(axis='y', labelcolor='orange')

    axs[2].set_xlabel('Time [s]')
    axs[2].set_title('Forces & Torques')

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    params = SimParams()
    sim = CamSystemSimulation(params)
    history = sim.run()
    plot_results(history)