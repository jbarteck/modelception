import mujoco
from mujoco import viewer

scene = """
<mujoco model = "test_scene">
    <worldbody>
        <geom type="plane" size="3 3 0.1" rgba="0.9 0.9 0.9 1"/>
        <body name="table" pos="0 0 0.98">
            <geom name="top" type="box" size="0.5 0.5 0.02" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_fl" type="box" size="0.03 0.03 0.48" pos=" 0.45  0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_fr" type="box" size="0.03 0.03 0.48" pos=" 0.45 -0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_bl" type="box" size="0.03 0.03 0.48" pos="-0.45  0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_br" type="box" size="0.03 0.03 0.48" pos="-0.45 -0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
        </body>
        <body name="block" pos="0 0 1.05">
            <geom type="box" size="0.05 0.05 0.05" rgba="0.2 0.5 0.8 1"/>
        </body>
    </worldbody>
</mujoco>
"""

def main():
    model = mujoco.MjModel.from_xml_string(scene)
    data = mujoco.MjData(model)

    print("Opening viewer")
    with viewer.launch_passive(model, data) as view:
        view.cam.lookat[:] = (0.0, 0.0, 1.0)
        view.cam.distance = 2.0
        view.cam.elevation = -40.0
        while view.is_running():
            view.sync()
    print("Viewer closed")

main() # main block implemented to avoid stuck terminal error