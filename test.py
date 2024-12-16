import mujoco
import numpy as np
import time
from mujoco.viewer import launch

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path("arm_1.xml")  # Ensure your XML file name matches
data = mujoco.MjData(model)

# Simulation duration
simulation_duration = 10  # seconds
start_time = time.time()

# Launch the viewer
with launch(model, data) as viewer:
    print("Simulation started! Watch the elbow bend from 180° to 0°.")

    while time.time() - start_time < simulation_duration:
        sim_time = time.time() - start_time

        # Control logic for the elbow joint:
        # Smooth sinusoidal motion from 180° (straight) to 0° (contracted)
        ctrl_value = 90 * (1 - np.cos(2 * np.pi * sim_time / 3))  # Ranges from 0 to 180°
        data.ctrl[0] = ctrl_value  # Motor control value

        # Step the simulation
        mujoco.mj_step(model, data)

        # Render the simulation in real-time
        viewer.sync()
