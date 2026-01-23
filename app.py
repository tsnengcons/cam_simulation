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

<<<<<<< HEAD
# ==========================================
# 3. SIDEBAR: PARAMETER CONTROL
# ==========================================

st.sidebar.markdown("## ⚙️ Simulation Parameters")

# --- INSTANTIATE DEFAULT PARAMS ---
defaults = SimParams()
=======
with st.sidebar.expander("🔩 Springs & Constraints", expanded=True):
    st.markdown("**Vertical Spring:**")
    k_vert = st.number_input("Vert. Stiffness k [N/m]", value=4100.0)
    preload = st.number_input("Preload Force [N]", value=4.6)
    y_max = st.number_input("Max Vert. Travel [m]", value=0.001, format="%.4f")
    
    st.markdown("**Torsion Spring (Wheel-Axis):**")
    k_tor = st.number_input("Torsion k [Nm/rad]", value=0.18)
    c_tor = st.number_input("Torsion Damping c", value=0.001, format="%.4f")
    
    st.markdown("**Horizontal Latch:**")
    L_latch = st.number_input("Latch Distance [m]", value=0.006, format="%.4f")
    F_release = st.number_input("Latch Release Force [N]", value=0.0)
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

# --- TAB 1: PHYSICAL PARAMETERS ---
with st.sidebar.expander("A. Physical Properties (Geometry & Mass)", expanded=True):
    st.markdown("**1. Geometry**")
    R_wheel = st.number_input(r"Wheel Radius $R$ [m]", value=defaults.geo['R'], format="%.4f")
    R_sphere = st.number_input(r"Sphere Radius $R_s$ [m]", value=defaults.geo['R_s'], format="%.4f")
    N_lobes = st.number_input(r"Number of Lobes $N$", value=defaults.geo['N'], step=1)

<<<<<<< HEAD
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
=======
# --- RUN SIMULATION ---
if st.sidebar.button("▶️ Run Simulation", type="primary"):
    
    # 1. Setup Parameters
    params = SimParams()
    
    # Geometry
    params.geo['R'] = radius_wheel
    params.geo['R_s'] = radius_sphere
    params.geo['N'] = int(num_lobes)
    params.geo['d'] = d_gap
    
    # Physics
    params.mat_type = mat_type
    params.surf_type = surf_type
    params.mu_friction = SURFACES[surf_type]
    params.I_axis_factor = axis_inertia_fac
    
    # Input
    params.F_input_max = F_max
    params.t_input_ramp = t_ramp
    
    # Springs
    params.k_vert = k_vert
    params.F_preload = preload
    params.max_disp_y = y_max
    params.k_tor = k_tor
    params.c_tor = c_tor
    params.L_latch = L_latch
    params.F_latch_release = F_release
    
    # Time
    params.t_end = t_end
    params.dt = dt_options[dt_choice]
    params.save_gif = False 
    params.show_window = False
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46

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

<<<<<<< HEAD
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
=======
    # --- TAB 1: ANIMATION ---
    with tab1:
        st.subheader("System Animation")
        sim.init_anim_static()
        
        total_frames = 100 
        total_steps = len(history['t'])
        stride = max(1, total_steps // total_frames)
        
        ani = FuncAnimation(
            sim.fig_anim, 
            sim.update_anim_frame, 
            frames=range(0, total_steps, stride),
            blit=False
        )
        
        with st.spinner("Rendering Video Player..."):
            components = st.components.v1.html(ani.to_jshtml(), height=700, scrolling=False)

    # --- TAB 2: METRICS (UPDATED: NO SPHERE ANGLE) ---
    with tab2:
        st.subheader("Performance Metrics")
        
        fig, axs = plt.subplots(3, 1, figsize=(10, 14), sharex=True)
        t = np.array(history['t'])

        # Graph 1: Rotational Dynamics (Wheel & Axis only)
        axs[0].set_title("1. Rotational Dynamics")
        axs[0].plot(t, np.degrees(history['w_theta']), label='Wheel Angle', color='blue')
        axs[0].plot(t, np.degrees(history['a_theta']), label='Axis Angle', color='green', ls='--')
        # Sphere Angle removed from here
        axs[0].set_ylabel("Angle [deg]")
        axs[0].legend(loc='upper left')
        axs[0].grid(True, alpha=0.3)

        # Graph 2: Circle Motion (X & Y)
        axs[1].set_title("2. Circle Motion")
        axs[1].plot(t, np.array(history['c_x'])*1000, label='Horiz X', color='red')
        axs[1].set_ylabel("Horiz X [mm]", color='red')
        axs[1].tick_params(axis='y', labelcolor='red')
        
        ax1b = axs[1].twinx()
        y_disp = (np.array(history['c_y']) - history['c_y'][0])*1000
        ax1b.plot(t, y_disp, label='Vert Y', color='cyan')
        ax1b.set_ylabel("Vert Y [mm]", color='cyan')
        ax1b.tick_params(axis='y', labelcolor='cyan')
        
        lines, labels = axs[1].get_legend_handles_labels()
        lines2, labels2 = ax1b.get_legend_handles_labels()
        axs[1].legend(lines + lines2, labels + labels2, loc='upper left')
        axs[1].grid(True, alpha=0.3)

        # Graph 3: Horizontal Forces
        axs[2].set_title("3. Horizontal Forces on Circle")
        axs[2].plot(t, history['f_input'], label='Input Force (Right)', color='green')
        axs[2].plot(t, history['f_contact_x'], label='Reaction from Wheel', color='red', alpha=0.7)
        axs[2].set_ylabel("Force [N]")
        axs[2].set_xlabel("Time [s]")
        axs[2].legend(loc='upper left')
        axs[2].grid(True, alpha=0.3)
        
        st.pyplot(fig)

else:
    st.info("👈 Set parameters in the sidebar and click 'Run Simulation'.")
>>>>>>> 6d4625043e57a966ca1245861eb619397ed56c46
