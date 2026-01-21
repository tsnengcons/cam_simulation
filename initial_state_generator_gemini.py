# ic_generator.py
# -----------------------------------------------------------------------------
# Analytic Initial Condition Generator for Cam-Wheel System
# -----------------------------------------------------------------------------
# This module calculates the exact initial contact position of a circular follower
# nested inside a cam wheel pocket. It uses analytic geometry (angle bisectors)
# rather than settling simulations, ensuring zero overlap and perfect tangency.
#
# Public API:
#   get_initial_state_from_params(...)
#       Returns a dictionary containing the starting Center of Mass (C0),
#       Velocity (V0), and mass properties.

import numpy as np


# ==========================================
# 1. GEOMETRY GENERATION
# ==========================================

def rot2(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


def polar_xy(r, th):
    return np.array([r * np.cos(th), r * np.sin(th)])


def build_angles(N, alpha, theta0):
    # theta0 is the rotation phase of the wheel
    centers = theta0 + 2 * np.pi * np.arange(N) / N
    lefts = centers - alpha
    rights = centers + alpha
    # Sort angles to ensure sequential segment generation
    order = np.argsort(np.mod(lefts, 2 * np.pi))
    return centers[order], lefts[order], rights[order]


def sample_ramp_poly(theta_start, theta_end, R, d, samples=400):
    """Generates points for the curved 'rim' section."""
    thetas = np.linspace(theta_start, theta_end, samples)
    denom = max(theta_end - theta_start, 1e-12)
    u = (thetas - theta_start) / denom
    s = 0.5 * (1 - np.cos(np.pi * u))  # Smooth ease-in/out
    r = R - d * s
    pts = np.stack([r * np.cos(thetas), r * np.sin(thetas)], axis=1)
    return pts


def make_cam_segments_world(N, R, d, alpha_deg, theta0, ramp_samples=100, mirror_y=True):
    """
    Generates the list of line segments defining the wheel in World Coordinates.
    Returns list of dicts: {'p0': vec, 'p1': vec, 'tag': str, 'k': int}
    """
    alpha = np.deg2rad(alpha_deg)
    centers_b, lefts_b, rights_b = build_angles(N, alpha, theta0)

    segs = []

    # 1. Generate in Body Frame (assumes wheel rotation=0 for construction relative to angles)
    # Note: We actually build 'world' directly based on theta0

    for k in range(N):
        th_c = centers_b[k]
        th_rm = rights_b[k]
        th_nxt = centers_b[(k + 1) % N]

        # -- Straight Wall (Inner -> Outer) --
        # From inner radius (R-d) to outer rim (R)
        A_k = polar_xy(R - d, th_c)
        RM_k = polar_xy(R, th_rm)
        segs.append({'p0': A_k, 'p1': RM_k, 'k': k, 'tag': 'wall_R'})

        # -- Curved Rim (Outer -> Next Inner) --
        th0_r, th1_r = th_rm, th_nxt
        if th1_r <= th0_r: th1_r += 2 * np.pi

        ramp_pts = sample_ramp_poly(th0_r, th1_r, R, d, samples=ramp_samples)
        for i in range(len(ramp_pts) - 1):
            segs.append({'p0': ramp_pts[i], 'p1': ramp_pts[i + 1], 'k': k, 'tag': 'rim'})

    # 2. Apply Mirroring if needed (flips Y)
    if mirror_y:
        for s in segs:
            s['p0'] = np.array([-s['p0'][0], s['p0'][1]])
            s['p1'] = np.array([-s['p1'][0], s['p1'][1]])

    return segs


# ==========================================
# 2. ANALYTIC SOLVER (Corner Nesting)
# ==========================================

def solve_corner_nesting(segs, R_s):
    """
    Finds the 'Pocket' where a Rim segment meets a Wall segment, and calculates
    the center of a circle of radius R_s that is tangent to both.
    Returns the Center (C0) that is highest in Y (Top Dead Center).
    """
    best_C = np.array([0.0, 0.0])
    max_y = -float('inf')

    n_segs = len(segs)

    for i in range(n_segs):
        # We look for the transition: Previous=Rim -> Current=Wall
        # This is the "valley" vertex.
        prev_seg = segs[(i - 1) % n_segs]
        curr_seg = segs[i]

        if prev_seg['tag'] == 'rim' and curr_seg['tag'] == 'wall_R':
            vertex = curr_seg['p0']  # Start of wall (shared point)

            # Vector pointing BACK along the Rim (P1 -> P0)
            u = prev_seg['p0'] - prev_seg['p1']
            # Vector pointing FORWARD along the Wall (P0 -> P1)
            v = curr_seg['p1'] - curr_seg['p0']

            norm_u = np.linalg.norm(u)
            norm_v = np.linalg.norm(v)

            if norm_u < 1e-9 or norm_v < 1e-9:
                continue

            u_hat = u / norm_u
            v_hat = v / norm_v

            # --- Angle Bisector Calculation ---
            # cos(theta) = u . v
            cos_theta = np.clip(np.dot(u_hat, v_hat), -1.0, 1.0)
            theta = np.arccos(cos_theta)
            half_theta = theta / 2.0

            sin_half = np.sin(half_theta)

            if sin_half > 1e-6:
                # Distance from Vertex to Circle Center
                L = R_s / sin_half

                # Bisector direction vector (pointing out from vertex)
                bisector = u_hat + v_hat
                bisector_len = np.linalg.norm(bisector)

                if bisector_len > 1e-9:
                    bisector_hat = bisector / bisector_len
                    C_candidate = vertex + bisector_hat * L

                    # --- Selection Strategy ---
                    # We pick the solution with the highest Y value (assuming the
                    # simulation starts with the active cam near the top).
                    if C_candidate[1] > max_y:
                        max_y = C_candidate[1]
                        best_C = C_candidate

    return best_C


# ==========================================
# 3. MASS PROPERTIES
# ==========================================

def mass_inertia_circle(rho, R_s, t_s):
    """Solid disk: m = rho*pi*r^2*t, I = 0.5*m*r^2"""
    m = rho * np.pi * R_s ** 2 * t_s
    I = 0.5 * m * R_s ** 2
    return m, I


def mass_inertia_wheel_annulus(rho, R_outer, d, t_w):
    """Annular disk approximation."""
    R_inner = max(R_outer - d, 0.0)
    area = np.pi * (R_outer ** 2 - R_inner ** 2)
    m = rho * area * t_w
    I = 0.5 * m * (R_outer ** 2 + R_inner ** 2)
    return m, I


# ==========================================
# 4. PUBLIC ENTRYPOINT
# ==========================================

def get_initial_state_from_params(
        N, d, R_s, R,
        rho=7850.0, t_s=0.003, t_w=0.006,
        socket_angle_deg=9.0,
        theta0=np.pi / 2,
        mode=None  # 'mode' is kept for backward compat, but ignored
):
    """
    Generates the initial static state for the Cam-Wheel system.

    Parameters
    ----------
    N : int
        Number of cams/lobes.
    d : float
        Depth of the cam [m].
    R_s, R : float
        Radius of circle and wheel [m].
    socket_angle_deg : float
        Half-angle of the cam socket.
    theta0 : float
        Initial rotation of the wheel [rad].
        Defaults to pi/2 (12 o'clock).

    Returns
    -------
    dict
        {
            "C0": (x, y),       # Center of circle
            "V0": (0.0, 0.0),   # Initial velocity
            "geom": { ... },    # Segment data for plotting/collision
            "mass_props": { ... }
        }
    """

    # 1. Generate World Geometry
    # We reduce ramp_samples for the "solver" (physics engine needs density,
    # analytic solver only needs the vertices).
    # However, for drawing, we might want high res. We'll use 100.
    segs_w = make_cam_segments_world(
        N, R, d, float(socket_angle_deg), theta0,
        ramp_samples=100, mirror_y=True
    )

    # 2. Solve for Position
    C0_vec = solve_corner_nesting(segs_w, float(R_s))

    # 3. Mass Props
    m_s, I_s = mass_inertia_circle(rho, R_s, t_s)
    m_w, I_w = mass_inertia_wheel_annulus(rho, R, d, t_w)

    return {
        "C0": tuple(C0_vec),
        "V0": (0.0, 0.0),
        "geom": {
            "segs_w": segs_w,  # List of dicts {'p0', 'p1', 'tag', 'k'}
            "R_s": float(R_s),
            "params": {
                "N": int(N), "d": float(d), "R": float(R),
                "R_s": float(R_s), "theta0": float(theta0)
            }
        },
        "mass_props": {
            "circle": {"m": m_s, "I": I_s},
            "wheel": {"m": m_w, "I": I_w}
        }
    }