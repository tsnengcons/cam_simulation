# dynamics_sim_3d_sphere.py
# -----------------------------------------------------------------------------
# Core Physics Library for Cam-Sphere System
# -----------------------------------------------------------------------------
# Designed for integration with Streamlit GUI.
# Features:
# - 3D Sphere Physics (Mass, Inertia, Hertzian Contact)
# - Dual-Axis Plotting with Synchronized Animation Markers
# - Callback-based Progress Bar
# - Strict separation of Physical vs Numerical parameters

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.animation import FuncAnimation, PillowWriter
import initial_state_generator_gemini as ic_generator
import sys

# ==========================================
# 1. DATABASE
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
# 2. PARAMETERS CLASS
# ==========================================

class SimParams:
    def __init__(self):
        # -------------------------------------------------------
        # A. PHYSICAL PARAMETERS (Geometry, Mass, Springs)
        # -------------------------------------------------------
        self.geo = {
<<<<<<< HEAD
            "N": 7,  # Number of lobes
            "d": 0.001,  # Gap parameter
            "R_s": 0.002,  # Sphere Radius [m]
            "R": 0.008,  # Wheel Radius [m]
=======
            "N": 7,
            "d": 0.001,
            "R_s": 0.002,     # Sphere Radius
            "R": 0.008,
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
            "socket_angle_deg": 5,
            "theta0": np.pi / 2
        }

        # Mass & Inertia
        self.I_axis_factor = 0.1  # Inertia scaling for the axis

        # Input Force (The "Push")
        self.F_input_max = 8.0  # [N]
        self.t_input_ramp = 0.1  # [s] Time to ramp up force

        # Springs & Constraints
        self.k_vert = 4100.0  # [N/m] Vertical Spring Stiffness
        self.F_preload = 4.6  # [N] Downward Preload Force
        self.max_disp_y = 0.001  # [m] Ceiling limit (Hard stop)

        self.k_tor = 0.18  # [Nm/rad] Torsion Spring Stiffness
        self.c_tor = 0.001  # Torsion Damping

        self.L_latch = 0.006  # [m] Latch horizontal distance
        self.F_latch_release = 0.0  # [N] Force required to release latch

        # Materials
        self.mat_type = 'Steel'
        self.surf_type = 'Greased'
        self.mu_friction = SURFACES[self.surf_type]

<<<<<<< HEAD
        # -------------------------------------------------------
        # B. NUMERICAL & SIMULATION PARAMETERS
        # -------------------------------------------------------
        self.dt = 2.0e-6  # [s] Time step (Fixed for stability)
        self.t_end = 0.015  # [s] Total duration
=======
        # --- Limits ---
        self.max_vel_rot = 6000.0  # rad/s
        self.max_force = 2000.0    # N
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        # Limits (Safety Clamps)
        self.max_vel_rot = 6000.0  # [rad/s]
        self.max_force = 2000.0  # [N]

        # -------------------------------------------------------
        # C. VISUALIZATION SETTINGS (Defaults)
        # -------------------------------------------------------
        self.gif_fps = 30  # Frames per second for playback
        self.anim_duration = 8.0  # [s] Desired playback length


# ==========================================
# 3. MATH HELPERS
# ==========================================

def rot2(theta):
    """2D Rotation Matrix"""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])

def get_closest_point_on_segment(P, A, B):
    """Finds closest point on line segment AB to point P"""
    AP = P - A
    AB = B - A
    ab2 = np.dot(AB, AB)
    if ab2 < 1e-12: return A
    t = np.dot(AP, AB) / ab2
    t = np.clip(t, 0.0, 1.0)
    return A + t * AB

<<<<<<< HEAD

def moving_average(data, window_size=50):
    """Smoothes data for plotting"""
    if len(data) < window_size: return data
    return np.convolve(data, np.ones(window_size) / window_size, mode='same')


=======
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
# ==========================================
# 4. SIMULATION ENGINE
# ==========================================

class CamSystemSimulation:
    def __init__(self, params):
        self.p = params
<<<<<<< HEAD
        self._init_geometry()
        self._init_physics()
        self._init_state()

        # Placeholders for plotting artists
        self.fig = None
        self.lines = {}
        self.dots = {}

    def _init_geometry(self):
        """Generates the Cam Wheel Geometry"""
=======
        
        # --- Initialize Geometry ---
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        ic = ic_generator.get_initial_state_from_params(**self.p.geo)
        self.C0 = np.array(ic['C0'])
        self.segs_local = ic['geom']['segs_w']

        # Rotate segments to start at theta0
        R_inv = rot2(-self.p.geo['theta0'])
        pts = []
        for s in self.segs_local:
            s['p0'] = R_inv @ s['p0']
            s['p1'] = R_inv @ s['p1']
            pts.append(s['p0'])
        if len(self.segs_local) > 0:
            pts.append(self.segs_local[0]['p0'])
        self.wheel_pts_local = np.array(pts)
        self.ic_data = ic  # Store full IC data

    def _init_physics(self):
        """Calculates Mass, Inertia, and Stiffness"""
        # Sphere 3D Properties
        r_s = self.p.geo['R_s']
        vol_s = (4.0/3.0) * np.pi * (r_s**3)
        rho = MATERIALS[self.p.mat_type]['rho']
<<<<<<< HEAD
=======
        self.m_c = rho * vol_s
        self.I_c = 0.4 * self.m_c * (r_s**2)
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        self.m_c = rho * vol_s
        self.I_c = 0.4 * self.m_c * (r_s ** 2)  # Solid sphere inertia

        # Wheel Inertia
        self.I_w = self.ic_data['mass_props']['wheel']['I']
        self.I_a = self.I_w * self.p.I_axis_factor

        # Contact Stiffness Tuning (Stability Check)
        safety_factor = 0.15
        k_linear_stable = safety_factor * self.m_c / (self.p.dt ** 2)
        typ_pen = r_s * 0.01

        self.k_contact = k_linear_stable / np.sqrt(typ_pen)  # Hertzian K

        mat_props = MATERIALS[self.p.mat_type]
        self.c_contact = 2 * mat_props['zeta'] * np.sqrt(k_linear_stable * self.m_c)

    def _init_state(self):
        self.state = {
            'c_pos': self.C0.copy(),
            'c_vel': np.zeros(2),
<<<<<<< HEAD
            'c_theta': 0.0, 'c_omega': 0.0,
            'w_theta': self.p.geo['theta0'], 'w_omega': 0.0,
            'a_theta': self.p.geo['theta0'], 'a_omega': 0.0
=======
            'c_theta': 0.0,      
            'c_omega': 0.0,      
            'w_theta': self.p.geo['theta0'],
            'w_omega': 0.0,
            'a_theta': self.p.geo['theta0'],
            'a_omega': 0.0
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        }
        self.is_latched = False
<<<<<<< HEAD
        self.history = {k: [] for k in [
            't', 'c_x', 'c_y', 'c_theta', 'w_theta',
            'f_input', 'f_contact_x'
        ]}
=======
        # UPDATED HISTORY: Added specific force tracking
        self.history = {k: [] for k in
                        ['t', 'c_x', 'c_y', 'c_theta', 'c_omega', 'w_theta', 
                         'w_omega', 'a_theta', 'f_contact_x', 'f_input']}

        self.fig_anim = None
        self.ax_geo = None
        self.ln_wheel = None; self.ln_axis = None
        self.patch_circle = None; self.patch_stripe = None
        self.txt_info = None

    def tune_contact_physics(self):
        safety_factor = 0.15
        k_linear_stable = safety_factor * self.m_c / (self.p.dt ** 2)
        typ_pen = self.p.geo['R_s'] * 0.01
        self.k_contact = k_linear_stable / np.sqrt(typ_pen)

        mat_props = MATERIALS[self.p.mat_type]
        zeta = mat_props['zeta']
        self.c_contact = 2 * zeta * np.sqrt(k_linear_stable * self.m_c)
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

    def get_input_force(self, t):
        if t < self.p.t_input_ramp:
            return (self.p.F_input_max / self.p.t_input_ramp) * t
        return self.p.F_input_max

<<<<<<< HEAD
=======
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

>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
    def step(self, t):
        s = self.state
        p = self.p

<<<<<<< HEAD
        # --- Forces & Torques ---
        F_c = np.zeros(2)
        tau_c = 0.0
        tau_w = 0.0
        tau_a = 0.0
=======
        F_c_net = np.zeros(2)
        tau_c_net = 0.0
        tau_w_net = 0.0
        tau_a_net = 0.0
        
        # Track specific horizontal forces for graphing
        f_contact_x_val = 0.0
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        # 1. Input Force
        F_in = self.get_input_force(t)
        F_c[0] += F_in

        # 2. Vertical Spring
        dy = s['c_pos'][1] - self.C0[1]
        F_c[1] += (-p.k_vert * dy - p.F_preload)

        # 3. Torsion Spring
        d_theta = s['w_theta'] - s['a_theta']
        d_omega = s['w_omega'] - s['a_omega']
        tau_spring = -p.k_tor * d_theta - p.c_tor * d_omega
        tau_w += tau_spring
        tau_a -= tau_spring

<<<<<<< HEAD
        # 4. Contact Detection & Resolution
        R_mat_T = rot2(-s['w_theta'])
        C_local = R_mat_T @ s['c_pos']

        best_contact = None
        max_pen = -1e9
=======
        # 4. Contact
        contact = self.detect_contact()
        
        if contact:
            pen, n_vec, Q_world = contact
            r_w = Q_world 
            r_c = -p.geo['R_s'] * n_vec 
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        for seg in self.segs_local:
            Q_local = get_closest_point_on_segment(C_local, seg['p0'], seg['p1'])
            dist = np.linalg.norm(C_local - Q_local)
            pen = p.geo['R_s'] - dist

            if pen > 0 and pen > max_pen:
                max_pen = pen
                if dist < 1e-12:  # Handle center overlap
                    d = seg['p1'] - seg['p0']
                    n_local = np.array([-d[1], d[0]]) / np.linalg.norm(d)
                else:
                    n_local = (C_local - Q_local) / dist

                R_mat = rot2(s['w_theta'])
                best_contact = (pen, R_mat @ n_local, R_mat @ Q_local)

        f_react_x = 0.0
        if best_contact:
            pen, n, Q = best_contact
            r_w = Q
            r_c = -p.geo['R_s'] * n  # Vector from Circle Center to Contact

            # Velocities
            v_w = np.array([-s['w_omega'] * r_w[1], s['w_omega'] * r_w[0]])
            v_c_surf = s['c_vel'] + np.array([-s['c_omega'] * r_c[1], s['c_omega'] * r_c[0]])
            v_rel = v_c_surf - v_w

            # Normal Force (Hertzian)
            vn = np.dot(v_rel, n)
            Fn = max(0.0, self.k_contact * (pen ** 1.5) - self.c_contact * vn)
            Fn = min(Fn, p.max_force)
<<<<<<< HEAD
            F_N_vec = Fn * n

            # Friction
            t_vec = np.array([-n[1], n[0]])
            vt = np.dot(v_rel, t_vec)
            mu_eff = p.mu_friction * np.tanh(vt / 0.01)
            F_F_vec = -mu_eff * Fn * t_vec

            F_total = F_N_vec + F_F_vec
            F_c += F_total
            f_react_x = F_total[0]  # Store reaction for plotting

            # Torques
            tau_c += (r_c[0] * F_F_vec[1] - r_c[1] * F_F_vec[0])
            tau_w += (r_w[0] * (-F_total[1]) - r_w[1] * (-F_total[0]))

        # 5. Constraints (Walls/Latch)
        # Left Wall
=======
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
            tau_c_friction = r_c[0]*F_F_vec[1] - r_c[1]*F_F_vec[0]
            tau_c_net += tau_c_friction

            # Torque on Wheel
            tau_w_net += (r_w[0]*(-F_total_contact[1]) - r_w[1]*(-F_total_contact[0]))

        # 5. Constraints & Limits
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        if s['c_pos'][0] < self.C0[0]:
            s['c_pos'][0] = self.C0[0]
            if s['c_vel'][0] < 0: s['c_vel'][0] = 0
            if F_c[0] < 0: F_c[0] = 0

        # Floor / Ceiling
        ceil = self.C0[1] + p.max_disp_y
<<<<<<< HEAD
        floor = self.C0[1]
        if s['c_pos'][1] > ceil:
=======
        
        if curr_y < floor:
            s['c_pos'][1] = floor
            if s['c_vel'][1] < 0: s['c_vel'][1] = 0.0
            if abs(s['c_vel'][0]) > 1e-4:
                 drag = -5.0 * s['c_vel'][0] 
                 F_c_net[0] += drag
        elif curr_y > ceil:
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
            s['c_pos'][1] = ceil
            if s['c_vel'][1] > 0: s['c_vel'][1] = 0
        elif s['c_pos'][1] < floor:
            s['c_pos'][1] = floor
            if s['c_vel'][1] < 0: s['c_vel'][1] = 0
            # Simple floor friction
            F_c[0] -= 5.0 * s['c_vel'][0]

        # Latch
        dx = s['c_pos'][0] - self.C0[0]
<<<<<<< HEAD
        if not self.is_latched and dx >= p.L_latch:
            self.is_latched = True
            s['c_pos'][0] = self.C0[0] + p.L_latch
            s['c_vel'][0] = 0
            F_c[0] = 0

=======
        if not self.is_latched:
            if dx >= p.L_latch:
                self.is_latched = True
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0.0
                s['c_omega'] = 0.0
                F_c_net[0] = 0.0
        
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        if self.is_latched:
            if F_c[0] < -p.F_latch_release:
                self.is_latched = False
            else:
                s['c_pos'][0] = self.C0[0] + p.L_latch
                s['c_vel'][0] = 0
                F_c[0] = 0

<<<<<<< HEAD
        # 6. Integration (Semi-Implicit Euler)
        # Accel
        acc_c = F_c / self.m_c
        alp_c = tau_c / self.I_c
        alp_w = tau_w / self.I_w

        # Axis Sprag Clutch
        alp_a = tau_a / self.I_a
        if s['a_omega'] <= 1e-5 and tau_a < 0:
            alp_a = 0;
            s['a_omega'] = 0
=======
        # Integration
        acc_c = F_c_net / self.m_c
        alpha_c = tau_c_net / self.I_c
        alpha_w = tau_w_net / self.I_w
        alpha_a = tau_a_net / self.I_a
        
        if s['a_omega'] <= 1e-5 and tau_a_net < 0:
            alpha_a = 0.0
            s['a_omega'] = 0.0
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        # Update Vel
        s['c_vel'] += acc_c * p.dt
        s['c_omega'] += alp_c * p.dt
        s['w_omega'] += alp_w * p.dt
        s['a_omega'] += alp_a * p.dt

        # Safety
        s['w_omega'] = np.clip(s['w_omega'], -p.max_vel_rot, p.max_vel_rot)
<<<<<<< HEAD
        s['c_omega'] = np.clip(s['c_omega'], -p.max_vel_rot * 2, p.max_vel_rot * 2)
=======
        s['a_omega'] = np.clip(s['a_omega'], -p.max_vel_rot, p.max_vel_rot)
        s['c_omega'] = np.clip(s['c_omega'], -p.max_vel_rot*2, p.max_vel_rot*2)
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

        # Update Pos
        s['c_pos'] += s['c_vel'] * p.dt
        s['c_theta'] += s['c_omega'] * p.dt
        s['w_theta'] += s['w_omega'] * p.dt
        s['a_theta'] += s['a_omega'] * p.dt

        # 7. History
        self.history['t'].append(t)
        self.history['c_x'].append(s['c_pos'][0])
        self.history['c_y'].append(s['c_pos'][1])
        self.history['c_theta'].append(s['c_theta'])
        self.history['w_theta'].append(s['w_theta'])
        self.history['f_input'].append(F_in)
        self.history['f_contact_x'].append(f_react_x)

    def run(self, progress_callback=None):
        """
        Runs the full simulation loop.
        progress_callback: A function(0.0 to 1.0) to update a GUI progress bar.
        """
        steps = int(self.p.t_end / self.p.dt)
        t = 0.0

        # Pre-process smoothing for plots at the end?
        # No, just run physics here.

        for i in range(steps):
            self.step(t)
            t += self.p.dt

            # Update progress every 2%
            if i % (steps // 50) == 0:
                if progress_callback:
                    progress_callback(i / steps)
                else:
                    # Default console print
                    sys.stdout.write(f"\rSimulating: {i / steps * 100:.0f}%")
                    sys.stdout.flush()

        if progress_callback: progress_callback(1.0)
        print("\nDone.")
        return self.history

<<<<<<< HEAD
    # ==========================================
    # VISUALIZATION (Animation & Plots)
    # ==========================================

    def setup_animation_figure(self):
        """
        Creates the Figure, Axes, and Plot Artists for the animation.
        This allows the GUI to just call 'update' later.
        """
        # Create a layout: Left = Animation, Right = 2 stacked graphs
        self.fig = plt.figure(figsize=(14, 8))
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1, 1.2])

        # --- 1. Geometry View (Left) ---
        self.ax_geo = self.fig.add_subplot(gs[:, 0])
        self.ax_geo.set_aspect('equal')
        self.ax_geo.grid(True, alpha=0.3)
        self.ax_geo.set_title(f"Simulation Replay ({self.p.mat_type})")

        # Limits
        lim = 1.4 * self.p.geo['R']
        self.ax_geo.set_xlim(-lim, lim)
        self.ax_geo.set_ylim(-lim, lim)

        # Walls
=======
    def init_anim_static(self):
        self.fig_anim = plt.figure(figsize=(10, 8))
        self.ax_geo = self.fig_anim.add_subplot(111)
        self.ax_geo.set_aspect('equal')
        self.ax_geo.grid(True)
        self.ax_geo.set_xlim(-1.2*self.p.geo['R'], 1.2*self.p.geo['R'])
        self.ax_geo.set_ylim(-1.2*self.p.geo['R'], 1.2*self.p.geo['R'])
        
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        floor = self.C0[1]
        ceil = self.C0[1] + self.p.max_disp_y
        self.ax_geo.axhline(floor, c='k', ls='-', lw=1)
        self.ax_geo.axhline(ceil, c='r', ls='--', lw=1)
        self.ax_geo.axvline(self.C0[0] + self.p.L_latch, c='r', ls=':', label='Latch')

        # Artists
        self.ln_wheel, = self.ax_geo.plot([], [], 'k-', lw=1.5)
<<<<<<< HEAD
        self.patch_circle = Circle((0, 0), self.p.geo['R_s'], color='red', alpha=0.7)
=======
        self.ln_axis, = self.ax_geo.plot([], [], 'g--', lw=2)
        
        self.patch_circle = Circle((0, 0), radius=self.p.geo['R_s'], color='red', alpha=0.6)
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        self.ax_geo.add_patch(self.patch_circle)
        self.patch_stripe, = self.ax_geo.plot([], [], 'w-', lw=1.5)
        self.txt_time = self.ax_geo.text(0.05, 0.95, "", transform=self.ax_geo.transAxes)

<<<<<<< HEAD
        # --- PRE-CALCULATE DATA FOR PLOTS ---
        t = np.array(self.history['t'])
        cx = np.array(self.history['c_x']) * 1000  # mm
        cy = (np.array(self.history['c_y']) - self.C0[1]) * 1000  # mm
        fin = np.array(self.history['f_input'])
        # Filter reaction force
        freact = moving_average(np.array(self.history['f_contact_x']), window_size=100)

        # --- 2. Displacement Graph (Top Right) ---
        self.ax_disp = self.fig.add_subplot(gs[0, 1])
        self.ax_disp.set_title("Displacements")
        self.ax_disp.grid(True, alpha=0.3)

        # Dual Axis
        self.ax_disp_y = self.ax_disp.twinx()

        self.ax_disp.plot(t, cx, 'r-', label='Horiz (X) [mm]')
        self.ax_disp_y.plot(t, cy, 'c-', label='Vert (Y) [mm]')

        self.ax_disp.set_ylabel('Horizontal [mm]', color='r')
        self.ax_disp_y.set_ylabel('Vertical [mm]', color='c')
        self.ax_disp.tick_params(axis='y', labelcolor='r')
        self.ax_disp_y.tick_params(axis='y', labelcolor='c')

        # Markers (The Dynamic Red Dots)
        self.dot_disp_x, = self.ax_disp.plot([], [], 'ro', markeredgecolor='k', ms=8)
        self.dot_disp_y, = self.ax_disp_y.plot([], [], 'co', markeredgecolor='k', ms=8)

        # --- 3. Forces Graph (Bottom Right) ---
        self.ax_force = self.fig.add_subplot(gs[1, 1], sharex=self.ax_disp)
        self.ax_force.set_title("Forces")
        self.ax_force.set_xlabel("Time [s]")
        self.ax_force.grid(True, alpha=0.3)

        # Dual Axis
        self.ax_force_r = self.ax_force.twinx()

        self.ax_force.plot(t, fin, 'g-', label='Input Force [N]')
        self.ax_force_r.plot(t, freact, 'm-', label='Reaction (Filtered) [N]')

        self.ax_force.set_ylabel('Input Force [N]', color='g')
        self.ax_force_r.set_ylabel('Reaction Force [N]', color='m')
        self.ax_force.tick_params(axis='y', labelcolor='g')
        self.ax_force_r.tick_params(axis='y', labelcolor='m')

        # Markers
        self.dot_force_in, = self.ax_force.plot([], [], 'go', markeredgecolor='k', ms=8)
        self.dot_force_re, = self.ax_force_r.plot([], [], 'mo', markeredgecolor='k', ms=8)

        plt.tight_layout()
        return self.fig

    def get_anim_frame(self, frame_idx, total_frames):
        """
        Update function for FuncAnimation.
        Updates geometry AND graph markers.
        """
        # Map frame index to history index
        total_steps = len(self.history['t'])
        idx = int((frame_idx / total_frames) * total_steps)
        if idx >= total_steps: idx = total_steps - 1

        # 1. Update Geometry
=======
    def update_anim_frame(self, frame_idx):
        idx = frame_idx
        if idx >= len(self.history['t']): idx = len(self.history['t']) - 1
        
        t = self.history['t'][idx]
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
        w_theta = self.history['w_theta'][idx]
        c_theta = self.history['c_theta'][idx]
        c_pos = [self.history['c_x'][idx], self.history['c_y'][idx]]

        # Wheel
        R_w = rot2(w_theta)
        pts = (R_w @ self.wheel_pts_local.T).T
        self.ln_wheel.set_data(pts[:, 0], pts[:, 1])

        # Sphere
        self.patch_circle.center = c_pos
<<<<<<< HEAD
        R_c = rot2(c_theta)
        stripe = (R_c @ np.array([[0, 0], [self.p.geo['R_s'], 0]]).T).T + c_pos
        self.patch_stripe.set_data(stripe[:, 0], stripe[:, 1])

        self.txt_time.set_text(f"t={self.history['t'][idx]:.4f}s")

        # 2. Update Graph Markers (The "Shiny Red Dot")
        t_val = self.history['t'][idx]

        # Disp
        cx = self.history['c_x'][idx] * 1000
        cy = (self.history['c_y'][idx] - self.C0[1]) * 1000
        self.dot_disp_x.set_data([t_val], [cx])
        self.dot_disp_y.set_data([t_val], [cy])

        # Force
        fin = self.history['f_input'][idx]
        # For reaction, we need the smoothed value at this index.
        # Ideally we pre-calc smooth array. Doing on fly is slow but safe for now:
        # Optimization: We pre-calculated 'freact' in setup_animation_figure.
        # But we can't access local variables of that function easily unless stored self.
        # Hack: Just read raw for the dot, or re-smooth.
        # Better: Since dot is just one point, reading RAW reaction is actually better
        # to see the instantaneous spike, OR we read the smooth value.
        # Let's read the smooth value from the line data we plotted!

        # Access data directly from the line object we plotted
        # Line 0 in ax_force_r is the magenta line
        smooth_y_data = self.ax_force_r.lines[0].get_ydata()

        # We need to map 'idx' (physics step) to the 'plot data index'.
        # Since we plotted ALL steps, the indices match 1:1.
        fre = smooth_y_data[idx]

        self.dot_force_in.set_data([t_val], [fin])
        self.dot_force_re.set_data([t_val], [fre])

        return [self.ln_wheel, self.patch_circle, self.dot_disp_x]

    def create_animation_object(self):
        """Generates the FuncAnimation object for the GUI to render"""
        self.setup_animation_figure()

        # Calculate Frames
        total_steps = len(self.history['t'])
        fps = self.p.gif_fps
        duration = self.p.anim_duration
        total_frames = int(fps * duration)

        ani = FuncAnimation(
            self.fig,
            self.get_anim_frame,
            fargs=(total_frames,),
            frames=total_frames,
            blit=False  # Streamlit plays better with blit=False usually
        )
        return ani
=======
        
        r = self.p.geo['R_s']
        stripe_local = np.array([[0, 0], [r, 0]])
        R_c = rot2(c_theta)
        stripe_world = (R_c @ stripe_local.T).T + np.array(c_pos)
        self.patch_stripe.set_data(stripe_world[:, 0], stripe_world[:, 1])
        
        self.txt_info.set_text(f"t={t:.4f}s")
        return self.ln_wheel, self.patch_circle, self.patch_stripe
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

# ==========================================
# MAIN (For Local Testing)
# ==========================================
<<<<<<< HEAD
if __name__ == "__main__":
    params = SimParams()
    sim = CamSystemSimulation(params)

    print("Running Simulation...")
    sim.run()

    print("Generating Animation Window...")
    # For local test, show the interactive window
    ani = sim.create_animation_object()
    plt.show()
=======
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
    axs[1].plot(t, np.array(hist['c_x'])*1000, label='Horiz X', color='red')
    ax1b = axs[1].twinx()
    y_disp = (np.array(hist['c_y']) - hist['c_y'][0])*1000
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
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
