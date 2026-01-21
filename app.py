import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, HTMLWriter

# Import simulation
from dynamics_sim_3d_sphere import SimParams, CamSystemSimulation, MATERIALS, SURFACES

# --- PAGE CONFIGURATION ---
st.set_page_config(page_title="Cam-Sphere Dynamics", layout="wide")
st.title("⚙️ 3D Sphere Cam Dynamics Simulator")

# --- SIDEBAR: ALL PARAMETERS ---
st.sidebar.header("1. Simulation Parameters")

with st.sidebar.expander("📐 Geometry", expanded=True):
    radius_wheel = st.number_input("Wheel Radius (R) [m]", value=0.008, format="%.4f")
    radius_sphere = st.number_input("Sphere Radius (Rs) [m]", value=0.002, format="%.4f")
    num_lobes = st.number_input("Number of Lobes (N)", value=7, step=1)
    d_gap = st.number_input("Gap Parameter (d) [m]", value=0.001, format="%.4f")

with st.sidebar.expander("⚖️ Mass & Materials", expanded=False):
    mat_type = st.selectbox("Material", list(MATERIALS.keys()), index=0)
    surf_type = st.selectbox("Surface Condition", list(SURFACES.keys()), index=1)
    axis_inertia_fac = st.number_input("Axis Inertia Factor", value=0.1)

with st.sidebar.expander("⚡ Input Force", expanded=True):
    F_max = st.number_input("Max Input Force [N]", value=8.0)
    t_ramp = st.number_input("Ramp Time [s]", value=0.1)

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

with st.sidebar.expander("⏱️ Time Settings", expanded=False):
    t_end = st.number_input("Duration [s]", value=0.015, step=0.001, format="%.4f")
    dt_options = {"High Precision (2.0e-6)": 2.0e-6, "Fast (5.0e-6)": 5.0e-6}
    dt_choice = st.selectbox("Time Step", list(dt_options.keys()))

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

    # 2. Run Physics
    with st.spinner("Calculating Physics..."):
        sim = CamSystemSimulation(params)
        history = sim.run()
        st.success("Physics calculation complete!")

    # 3. Results Layout
    tab1, tab2 = st.tabs(["🎬 Animation (Replay)", "📈 Analysis Metrics"])

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
