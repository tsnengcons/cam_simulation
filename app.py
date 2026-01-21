import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, HTMLWriter
import io

# Import your simulation class
from dynamics_sim_3d_sphere import SimParams, CamSystemSimulation, MATERIALS, SURFACES

# --- PAGE CONFIGURATION ---
st.set_page_config(page_title="Cam-Sphere Dynamics", layout="wide")

# --- DOCUMENTATION & TITLE ---
st.title("⚙️ 3D Sphere Cam Dynamics Simulator")

with st.expander("📖 Documentation & Physics Model", expanded=False):
    st.markdown(r"""
    ### Physical Model
    This simulation models the interaction between a rotating cam wheel and a spherical follower.

    **Key Physics Features:**
    1.  **Hertzian Contact:** The contact stiffness is non-linear ($F \propto \delta^{3/2}$), simulating real elastic deformation.
    2.  **Rigid Body Dynamics:** The sphere has mass moment of inertia ($I = \frac{2}{5}mr^2$) and simulates spin-up/spin-down.
    3.  **Traction & Slip:** Friction is calculated based on the relative velocity of the contact patch. The sphere can roll or slide (burnout).

    **Equations:**
    * **Contact Force:** $F_N = k \cdot \delta^{1.5} - c \cdot v_{rel,n}$
    * **Friction:** $\mu \cdot F_N \cdot \tanh(v_{slip}/v_{thresh})$
    """)

# --- SIDEBAR: PARAMETERS ---
st.sidebar.header("1. Simulation Parameters")

# 1. Geometry
st.sidebar.subheader("Geometry")
radius_wheel = st.sidebar.number_input("Wheel Radius (R) [m]", value=0.008, format="%.4f")
radius_sphere = st.sidebar.number_input("Sphere Radius (Rs) [m]", value=0.002, format="%.4f")
num_lobes = st.sidebar.number_input("Number of Lobes (N)", value=7, step=1)

# 2. Physics
st.sidebar.subheader("Physics Properties")
mat_type = st.sidebar.selectbox("Material", list(MATERIALS.keys()), index=0)
surf_type = st.sidebar.selectbox("Surface Condition", list(SURFACES.keys()), index=1)
k_vert = st.sidebar.number_input("Vertical Spring (k) [N/m]", value=4100.0)
preload = st.sidebar.number_input("Preload Force [N]", value=4.6)

# 3. Simulation Settings
st.sidebar.subheader("Time Settings")
t_end = st.sidebar.number_input("Duration [s]", value=0.015, step=0.001, format="%.4f")
# High dt is unstable, so we limit the user's choice or keep it fixed in background
dt_options = {"High Precision (2.0e-6)": 2.0e-6, "Fast (5.0e-6)": 5.0e-6}
dt_choice = st.sidebar.selectbox("Time Step", list(dt_options.keys()))

# --- INITIALIZE SIMULATION ---
if st.sidebar.button("▶️ Run Simulation", type="primary"):

    # 1. Setup Parameters
    params = SimParams()

    # Overwrite params with GUI values
    params.geo['R'] = radius_wheel
    params.geo['R_s'] = radius_sphere
    params.geo['N'] = int(num_lobes)

    params.mat_type = mat_type
    params.surf_type = surf_type
    params.mu_friction = SURFACES[surf_type]

    params.k_vert = k_vert
    params.F_preload = preload

    params.t_end = t_end
    params.dt = dt_options[dt_choice]

    # Disable file saving for the web app (we render to memory)
    params.save_gif = False
    params.show_window = False
    params.animate = True

    # 2. Run Physics
    with st.spinner("Calculating Physics... (This may take a moment)"):
        sim = CamSystemSimulation(params)
        history = sim.run()  # This runs the physics loop
        st.success("Physics calculation complete!")

    # 3. Create Layout for Results
    tab1, tab2, tab3 = st.tabs(["🎬 Animation", "📈 Analysis Metrics", "📄 Raw Data"])

    # --- TAB 1: ANIMATION ---
    with tab1:
        st.subheader("System Animation")

        sim.init_anim_static()

        # Calculate stride for reasonable rendering time
        total_frames = 100
        total_steps = len(history['t'])
        stride = max(1, total_steps // total_frames)

        ani = FuncAnimation(
            sim.fig_anim,
            sim.update_anim_frame,
            frames=range(0, total_steps, stride),
            blit=False
        )

        with st.spinner("Rendering Animation..."):
            components = st.components.v1.html(ani.to_jshtml(), height=800, scrolling=True)

    # --- TAB 2: METRICS ---
    with tab2:
        st.subheader("Performance Metrics")

        fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        t = np.array(history['t'])

        # Plot 1: Displacement
        axs[0].set_title("Vertical Displacement")
        y_disp = (np.array(history['c_y']) - history['c_y'][0]) * 1000
        axs[0].plot(t, y_disp, color='cyan', label='Y Displacement')
        axs[0].set_ylabel("Disp [mm]")
        axs[0].grid(True, alpha=0.3)
        axs[0].legend()

        # Plot 2: Sphere Rotation (Spin)
        axs[1].set_title("Sphere Spin Speed")
        axs[1].plot(t, history['c_omega'], color='orange', label='Spin (rad/s)')
        axs[1].axhline(0, color='grey', lw=0.5)
        axs[1].set_ylabel("Omega [rad/s]")
        axs[1].grid(True, alpha=0.3)
        axs[1].legend()

        # Plot 3: Forces
        axs[2].set_title("Contact Forces")
        axs[2].plot(t, history['f_contact'], color='red', alpha=0.7, label='Contact Force')
        axs[2].set_ylabel("Force [N]")
        axs[2].set_xlabel("Time [s]")
        axs[2].grid(True, alpha=0.3)
        axs[2].legend()

        st.pyplot(fig)

    # --- TAB 3: DATA ---
    with tab3:
        st.dataframe(history)

else:
    st.info("👈 Adjust parameters in the sidebar and click 'Run Simulation' to start.")