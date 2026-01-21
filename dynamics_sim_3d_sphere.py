# dynamics_sim_3d_sphere.py
# -----------------------------------------------------------------------------
# Rigid Body Dynamics Simulation: 3D Sphere on 2D Cam
# -----------------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter
import initial_state_generator_gemini as ic_generator
import sys

# ==========================================
# 1. MATERIAL PROPERTIES
# ==========================================

MATERIALS = {
    'Steel': {'zeta': 0.02, 'E': 200e9, 'nu': 0.30, 'rho': 7800},
    'Aluminum': {'zeta': 0.05, 'E': 69e9, 'nu': 0.33, 'rho': 2700},
    'Nylon': {'zeta': 0.30, 'E': 2e9, 'nu': 0.40, 'rho': 1150},
    'Rubber': {'zeta': 0.90, 'E': 0.1e9, 'nu': 0.48, 'rho': 1500},
}

SURFACES = {
    'Dry': 0.40,
    'Greased': 0.10,
    'Oiled': 0.05,
    'Tacky': 0.80,
}


# ==========================================
# 2. PARAMETERS
# ==========================================

class SimParams:
    def __init__(self):
        # --- Visualization ---
        self.animate = True
        self.save_gif = True
        self.show_window = True
        self.gif_real_time_length = 8.0
        self.gif_fps = 30

        # --- Time ---
        self.dt = 2.0e-6
        self.t_end = 0.01

        # --- Geometry ---
        self.geo = {
            "N": 7,
            "d": 0.001,
            "R_s": 0.002,  # Sphere Radius
            "R": 0.008,
            "socket_angle_deg": 5,
            "theta0": np.pi / 2
        }

        # --- Mass Properties (Sphere) ---
        self.I_axis_factor = 0.1

        # --- Input Force ---
        self.F_input_max = 8.0
        self.t_input_ramp = 0.1

        # --- Vertical Spring ---
        self.k_vert = 4100.0
        self.F_preload = 4.6
        self.max_disp_y = 0.001  # Hard stop

        # --- Horizontal Latch ---
        self.L_latch = 0.006
        self.F_latch_release = 0.0

        # --- Torsion Spring ---
        self.k_tor = 0.18
        self.c_tor = 0.001

        # --- Materials ---
        self.mat_type = 'Steel'
        self.surf_type = 'Greased'
        self.mu_friction = SURFACES[self.surf_type]

        # --- Limits ---
        self.max_vel_rot = 6000.0  # rad/s
        self.max_force = 2000.0  # N

        # --- Derived ---
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

        # --- Initialize Geometry ---
        ic = ic_generator.get_initial_state_from_params(**self.p.geo)
        self.C0 = np.array(ic['C0'])
        self.segs_local = ic['geom']['segs_w']

        R_inv = rot2(-self.p.geo['theta0'])
        pts = []
        for s in self.segs_local:
            s['p0'] = R_inv @ s['p0']
            s['p1'] = R_inv @ s['p1']
            pts.append(s['p0'])
        if len(self.segs_local) > 0:
            pts.append(self.segs_local[0]['p0'])
        self.wheel_pts_local = np.array(pts)

        # --- Mass Properties ---
        r_s = self.p.geo['R_s']
        vol_s = (4.0 / 3.0) * np.pi * (r_s ** 3)
        rho = MATERIALS[self.p.mat_type]['rho']
        self.m_c = rho * vol_s
        self.I_c = 0.4 * self.m_c * (r_s ** 2)

        self.I_w = ic['mass_props']['wheel']['I']
        self.I_a = self.I_w * self.p.I_axis_factor

        self.tune_contact_physics()

        # --- State Vector ---
        self.state = {
            'c_pos': self.C0.copy(),
            'c_vel': np.zeros(2),
            'c_theta': 0.0,
            'c_omega': 0.0,
            'w_theta': self.p.geo['theta0'],
            'w_omega': 0.0,
            'a_theta': self.p.geo['theta0'],
            'a_omega': 0.0
        }

        self.is_latched = False
        # UPDATED HISTORY: Added specific force tracking
        self.history = {k: [] for k in
                        ['t', 'c_x', 'c_y', 'c_theta', 'c_omega', 'w_theta',
                         'w_omega', 'a_theta', 'f_contact_x', 'f_input']}

        self.fig_anim = None
        self.ax_geo = None
        self.ln_wheel = None;
        self.ln_axis = None
        self.patch_circle = None;
        self.patch_stripe = None
        self.txt_info = None

    def tune_contact_physics(self):
        safety_factor = 0.15
        k_linear_stable = safety_factor * self.m_c / (self.p.dt ** 2)
        typ_pen = self.p.geo['R_s'] * 0.01
        self.k_contact = k_linear_stable / np.sqrt(typ_pen)

        mat_props = MATERIALS[self.p.mat_type]
        zeta = mat_props['zeta']
        self.c_contact = 2 * zeta * np.sqrt(k_linear_stable * self.m_c)

    def get_input_force(self, t):
        if t < self.p.t_input_ramp:
            return (self.p.F_input_max / self.p.t_input_ramp) * t
        return self.p.F_input_max

    def detect_contact(self):
        R_mat_T = rot2(-self.state['w_theta'])
        C_local = R_mat_T @ self.state['c_pos']
        max_pen = -1e9
        best_contact = None

        for seg in self.segs_local:
            P0, P1 = seg['p0'], seg['p1']
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
        tau_c_net = 0.0
        tau_w_net = 0.0
        tau_a_net = 0.0

        # Track specific horizontal forces for graphing
        f_contact_x_val = 0.0

        # 1. Input Force
        F_in = self.get_input_force(t)
        F_c_net[0] += F_in

        # 2. Vertical Spring
        dy = s['c_pos'][1] - self.C0[1]
        F_vert = -p.k_vert * dy - p.F_preload
        F_c_net[1] += F_vert

        # 3. Torsion Spring
        d_theta = s['w_theta'] - s['a_theta']
        d_omega = s['w_omega'] - s['a_omega']
        tau_spring = -p.k_tor * d_theta - p.c_tor * d_omega
        tau_w_net += tau_spring
        tau_a_net -= tau_spring

        # 4. Contact
        contact = self.detect_contact()

        if contact:
            pen, n_vec, Q_world = contact
            r_w = Q_world
            r_c = -p.geo['R_s'] * n_vec

            v_w_surf = np.array([-s['w_omega'] * r_w[1], s['w_omega'] * r_w[0]])
            v_rot_c = np.array([-s['c_omega'] * r_c[1], s['c_omega'] * r_c[0]])
            v_c_surf = s['c_vel'] + v_rot_c
            v_rel = v_c_surf - v_w_surf
            v_rel_n = np.dot(v_rel, n_vec)

            # Normal Force
            Fn = max(0.0, self.k_contact * (pen ** 1.5) - self.c_contact * v_rel_n)
            Fn = min(Fn, p.max_force)
            F_N_vec = Fn * n_vec

            # Friction Force
            t_vec = np.array([-n_vec[1], n_vec[0]])
            v_slip = np.dot(v_rel, t_vec)
            v_thresh = 0.01
            friction_coeff = p.mu_friction * np.tanh(v_slip / v_thresh)
            F_f_mag = friction_coeff * Fn
            F_F_vec = -F_f_mag * t_vec

            # Total Contact Force on Circle
            F_total_contact = F_N_vec + F_F_vec
            F_c_net += F_total_contact

            # Save Horizontal component (Reaction Force)
            f_contact_x_val = F_total_contact[0]

            # Torque on Circle
            tau_c_friction = r_c[0] * F_F_vec[1] - r_c[1] * F_F_vec[0]
            tau_c_net += tau_c_friction

            # Torque on Wheel
            tau_w_net += (r_w[0] * (-F_total_contact[1]) - r_w[1] * (-F_total_contact[0]))

        # 5. Constraints & Limits
        if s['c_pos'][0] < self.C0[0]:
            s['c_pos'][0] = self.C0[0]
            if s['c_vel'][0] < 0: s['c_vel'][0] = 0.0
            if F_c_net[0] < 0: F_c_net[0] = 0.0

        curr_y = s['c_pos'][1]
        floor = self.C0[1]
        ceil = self.C0[1] + p.max_disp_y

        if curr_y < floor:
            s['c_pos'][1] = floor
            if s['c_vel'][1] < 0: s['c_vel'][1] = 0.0
            if abs(s['c_vel'][0]) > 1e-4:
                drag = -5.0 * s['c_vel'][0]
                F_c_net[0] += drag
        elif curr_y > ceil:
            s['c_pos'][1] = ceil
            if s['c_vel'][1] > 0: s['c_vel'][1] = 0.0

        # Latch
        dx = s['c_pos'][0] - self.C0[0]
        if not self.is_latched:
            if dx >= p.L_latch:
                self.is_latched = True
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0.0
                s['c_omega'] = 0.0
                F_c_net[0] = 0.0

        if self.is_latched:
            if F_c_net[0] < -p.F_latch_release:
                self.is_latched = False
            else:
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0.0
                s['c_omega'] = 0.0
                F_c_net[0] = 0.0

        # Integration
        acc_c = F_c_net / self.m_c
        alpha_c = tau_c_net / self.I_c
        alpha_w = tau_w_net / self.I_w
        alpha_a = tau_a_net / self.I_a

        if s['a_omega'] <= 1e-5 and tau_a_net < 0:
            alpha_a = 0.0
            s['a_omega'] = 0.0

        s['c_vel'] += acc_c * p.dt
        s['c_omega'] += alpha_c * p.dt
        s['w_omega'] += alpha_w * p.dt
        s['a_omega'] += alpha_a * p.dt

        # Safety Clamps
        s['w_omega'] = np.clip(s['w_omega'], -p.max_vel_rot, p.max_vel_rot)
        s['a_omega'] = np.clip(s['a_omega'], -p.max_vel_rot, p.max_vel_rot)
        s['c_omega'] = np.clip(s['c_omega'], -p.max_vel_rot * 2, p.max_vel_rot * 2)

        s['c_pos'] += s['c_vel'] * p.dt
        s['c_theta'] += s['c_omega'] * p.dt
        s['w_theta'] += s['w_omega'] * p.dt
        s['a_theta'] += s['a_omega'] * p.dt

        # Save History
        self.history['t'].append(t)
        self.history['c_x'].append(s['c_pos'][0])
        self.history['c_y'].append(s['c_pos'][1])
        self.history['c_theta'].append(s['c_theta'])
        self.history['c_omega'].append(s['c_omega'])
        self.history['w_theta'].append(s['w_theta'])
        self.history['w_omega'].append(s['w_omega'])
        self.history['a_theta'].append(s['a_theta'])
        self.history['f_input'].append(F_in)
        self.history['f_contact_x'].append(f_contact_x_val)

    def run(self):
        steps = int(self.p.t_end / self.p.dt)
        t = 0.0
        for i in range(steps):
            self.step(t)
            t += self.p.dt
        return self.history

    def init_anim_static(self):
        self.fig_anim = plt.figure(figsize=(10, 8))
        self.ax_geo = self.fig_anim.add_subplot(111)
        self.ax_geo.set_aspect('equal')
        self.ax_geo.grid(True)
        self.ax_geo.set_xlim(-1.2 * self.p.geo['R'], 1.2 * self.p.geo['R'])
        self.ax_geo.set_ylim(-1.2 * self.p.geo['R'], 1.2 * self.p.geo['R'])

        floor = self.C0[1]
        ceil = self.C0[1] + self.p.max_disp_y
        self.ax_geo.axhline(floor, color='b', ls=':', alpha=0.5)
        self.ax_geo.axhline(ceil, color='r', ls=':', alpha=0.5)

        self.ln_wheel, = self.ax_geo.plot([], [], 'k-', lw=1.5)
        self.ln_axis, = self.ax_geo.plot([], [], 'g--', lw=2)

        self.patch_circle = Circle((0, 0), radius=self.p.geo['R_s'], color='red', alpha=0.6)
        self.ax_geo.add_patch(self.patch_circle)
        self.patch_stripe, = self.ax_geo.plot([], [], 'w-', lw=2)
        self.txt_info = self.ax_geo.text(0.02, 0.95, '', transform=self.ax_geo.transAxes)

    def update_anim_frame(self, frame_idx):
        idx = frame_idx
        if idx >= len(self.history['t']): idx = len(self.history['t']) - 1

        t = self.history['t'][idx]
        w_theta = self.history['w_theta'][idx]
        c_pos = [self.history['c_x'][idx], self.history['c_y'][idx]]
        c_theta = self.history['c_theta'][idx]

        R_mat = rot2(w_theta)
        pts = (R_mat @ self.wheel_pts_local.T).T
        self.ln_wheel.set_data(pts[:, 0], pts[:, 1])

        self.patch_circle.center = c_pos

        r = self.p.geo['R_s']
        stripe_local = np.array([[0, 0], [r, 0]])
        R_c = rot2(c_theta)
        stripe_world = (R_c @ stripe_local.T).T + np.array(c_pos)
        self.patch_stripe.set_data(stripe_world[:, 0], stripe_world[:, 1])

        self.txt_info.set_text(f"t={t:.4f}s")
        return self.ln_wheel, self.patch_circle, self.patch_stripe


# ==========================================
# 5. LOCAL PLOTTING (Matched to Request)
# ==========================================
def plot_results(hist):
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
    t = hist['t']

    # 1. Rotational Dynamics
    axs[0].set_title("1. Rotational Dynamics")
    axs[0].plot(t, np.degrees(hist['w_theta']), label='Wheel Angle', color='blue')
    axs[0].plot(t, np.degrees(hist['a_theta']), label='Axis Angle', color='green', ls='--')
    axs[0].plot(t, np.degrees(hist['c_theta']), label='Circle Rotation', color='orange')
    axs[0].set_ylabel('Angle [deg]')
    axs[0].legend(loc='upper left')
    axs[0].grid(True)

    # 2. Circle Motion (X & Y)
    axs[1].set_title("2. Circle Motion")
    axs[1].plot(t, np.array(hist['c_x']) * 1000, label='Horiz X', color='red')
    ax1b = axs[1].twinx()
    y_disp = (np.array(hist['c_y']) - hist['c_y'][0]) * 1000
    ax1b.plot(t, y_disp, label='Vert Y', color='cyan')
    axs[1].set_ylabel('Horiz X [mm]', color='red')
    ax1b.set_ylabel('Vert Y [mm]', color='cyan')
    lines, labels = axs[1].get_legend_handles_labels()
    lines2, labels2 = ax1b.get_legend_handles_labels()
    axs[1].legend(lines + lines2, labels + labels2, loc='upper left')
    axs[1].grid(True)

    # 3. Horizontal Forces (Input vs Reaction)
    axs[2].set_title("3. Horizontal Forces on Circle")
    axs[2].plot(t, hist['f_input'], label='Input Force (Right)', color='green')
    axs[2].plot(t, hist['f_contact_x'], label='Reaction from Wheel', color='red', alpha=0.7)
    axs[2].set_ylabel('Force [N]')
    axs[2].set_xlabel('Time [s]')
    axs[2].legend(loc='upper left')
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    params = SimParams()
    sim = CamSystemSimulation(params)
    history = sim.run()
    plot_results(history)