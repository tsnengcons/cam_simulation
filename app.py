import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
import streamlit.components.v1 as components

# Import the physics engine
from dynamics_sim_3d_sphere import SimParams, CamSystemSimulation, MATERIALS, SURFACES

# ==========================================
# 1. PAGE CONFIGURATION
# ==========================================
st.set_page_config(
    page_title="Cam-Sphere Contact Dynamics",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for "Scientific" look (Clean fonts, justified text)
st.markdown("""
<style>
    .reportview-container {
        background: #f0f2f6;
    }
    .main-header {
        font-family: "Helvetica Neue", Helvetica, Arial, sans-serif;
        font-weight: 700;
        color: #333;
    }
    .scientific-text {
        font-family: "Georgia", serif;
        font-size: 16px;
        line-height: 1.6;
        color: #444;
        text-align: justify;
    }
</style>
""", unsafe_allow_html=True)

# ==========================================
# 2. HEADER & DESCRIPTION
# ==========================================

st.markdown('<h1 class="main-header">Rigid Body Dynamics: Cam-Sphere Interaction Model</h1>', unsafe_allow_html=True)

st.markdown("""
<div class="scientific-text">
    This application simulates the coupled <b>3D contact mechanics</b> of a spherical follower interacting with a rotating cam wheel. 
    The model integrates the equations of motion for a multi-body system constrained by non-linear <b>Hertzian contact stiffness</b> and 
    Coulomb friction.
    <br><br>
    <b>Governing Physics:</b>
    <ul>
        <li><b>Contact Mechanics:</b> Normal force $F_N$ is derived from Hertz's Law ($F \propto \delta^{3/2}$), accounting for material elasticity ($E, \\nu$).</li>
        <li><b>Rotational Dynamics:</b> The sphere is modeled as a rigid body with moment of inertia $I = \\frac{2}{5}mr^2$, allowing for slip-stick transitions (traction vs. sliding).</li>
        <li><b>Coupled Constraints:</b> The system solves for the dynamic equilibrium of the driver (sphere), the cam wheel, and the sprag-clutched axis.</li>
    </ul>
</div>
<hr>
""", unsafe_allow_html=True)

# ==========================================
# 3. SIDEBAR: PARAMETER CONTROL
# ==========================================

st.sidebar.markdown("## ⚙️ Simulation Parameters")

# --- INSTANTIATE DEFAULT PARAMS ---
defaults = SimParams()

# --- TAB 1: PHYSICAL PARAMETERS ---
with st.sidebar.expander("A. Physical Properties (Geometry & Mass)", expanded=True):
    st.markdown("**1. Geometry**")
    R_wheel = st.number_input(r"Wheel Radius $R$ [m]", value=defaults.geo['R'], format="%.4f")
    R_sphere = st.number_input(r"Sphere Radius $R_s$ [m]", value=defaults.geo['R_s'], format="%.4f")
    N_lobes = st.number_input(r"Number of Lobes $N$", value=defaults.geo['N'], step=1)

    st.markdown("**2. Material Selection**")
    mat_name = st.selectbox("Material Type", list(MATERIALS.keys()), index=0)
    surf_name = st.selectbox("Surface Condition", list(SURFACES.keys()), index=1)

    st.markdown("**3. Mass & Inertia**")
    axis_inertia = st.number_input("Axis Inertia Factor (relative to Wheel)", value=defaults.I_axis_factor)

with st.sidebar.expander("B. Constitutive Forces (Springs & Input)", expanded=False):
    st.markdown("**1. Vertical Constraint**")
    k_vert = st.number_input(r"Vertical Stiffness $k_{vert}$ [N/m]", value=defaults.k_vert)
    preload = st.number_input(r"Preload Force $F_{pre}$ [N]", value=defaults.F_preload)

    st.markdown("**2. Torsional Constraint**")
    k_tor = st.number_input(r"Torsion Stiffness $k_{tor}$ [Nm/rad]", value=defaults.k_tor)
    c_tor = st.number_input(r"Torsion Damping $c_{tor}$", value=defaults.c_tor, format="%.4f")

    st.markdown("**3. Input Forcing**")
    F_in_max = st.number_input(r"Max Input Force [N]", value=defaults.F_input_max)
    t_ramp = st.number_input(r"Ramp Time [s]", value=defaults.t_input_ramp)

# --- TAB 2: NUMERICAL PARAMETERS ---
with st.sidebar.expander("C. Numerical Solver Settings", expanded=False):
    st.info("⚠️ High precision requires smaller time steps but increases computation time.")

    t_end = st.number_input("Simulation Duration [s]", value=0.015, step=0.005, format="%.4f")

    # Selection for dt to prevent user entering unstable values
    dt_map = {
        "Standard (2.0e-6 s)": 2.0e-6,
        "High Precision (1.0e-6 s)": 1.0e-6,
        "Coarse (5.0e-6 s)": 5.0e-6
    }
    dt_selection = st.selectbox("Time Step $\Delta t$", list(dt_map.keys()), index=0)
    dt_val = dt_map[dt_selection]

    st.markdown("---")
    anim_duration = st.slider("Playback Duration [s]", 5.0, 20.0, 8.0)

# ==========================================
# 4. EXECUTION LOGIC
# ==========================================

# Initialize Session State for the Animation object
if 'ani_html' not in st.session_state:
    st.session_state.ani_html = None


def run_simulation():
    # 1. Update Parameters Object
    p = SimParams()
    p.geo['R'] = R_wheel
    p.geo['R_s'] = R_sphere
    p.geo['N'] = int(N_lobes)

    p.mat_type = mat_name
    p.surf_type = surf_name
    p.mu_friction = SURFACES[surf_name]
    p.I_axis_factor = axis_inertia

    p.k_vert = k_vert
    p.F_preload = preload
    p.k_tor = k_tor
    p.c_tor = c_tor
    p.F_input_max = F_in_max
    p.t_input_ramp = t_ramp

    p.t_end = t_end
    p.dt = dt_val
    p.anim_duration = anim_duration

    # 2. Run Physics Engine
    sim = CamSystemSimulation(p)

    # Progress Bar Container
    progress_bar = st.progress(0)
    status_text = st.empty()

    def update_prog(val):
        progress_bar.progress(val)
        status_text.text(f"Integrating Equations of Motion... {val * 100:.0f}%")

    history = sim.run(progress_callback=update_prog)
    status_text.text("Rendering Visualization... (This may take a moment)")

    # 3. Generate Animation
    # We generate the HTML string once and store it in session state
    ani = sim.create_animation_object()
    st.session_state.ani_html = ani.to_jshtml()

    # Clean up
    status_text.empty()
    progress_bar.empty()
    plt.close(sim.fig)  # Close plt figure to free memory


# Big Run Button
if st.sidebar.button("▶️ Initialize & Run Simulation", type="primary"):
    run_simulation()

# ==========================================
# 5. RESULTS DISPLAY
# ==========================================

if st.session_state.ani_html:
    st.success("Simulation Converged. Visualization Ready.")

    # Display the JSHTML Animation
    # height=900 to accommodate the large Matplotlib figure (Animation + 2 Graphs)
    components.html(st.session_state.ani_html, height=1000, scrolling=False)

    st.info(
        "💡 **Interaction:** Use the slider below the graph to scrub through time. The shiny red dots on the right indicate the instantaneous state of the system.")

else:
    # Placeholder when no sim has run
    st.markdown("""
    <div style="background-color: #e8f4f8; padding: 20px; border-radius: 10px; border-left: 5px solid #00aaff;">
        <h4>Ready to Simulate</h4>
        <p>Adjust the parameters in the sidebar and click <b>"Initialize & Run Simulation"</b> to begin calculations.</p>
    </div>
    """, unsafe_allow_html=True)