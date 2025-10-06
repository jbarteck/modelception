import mujoco
from mujoco import viewer
from PIL import Image
import time

scene = """
<mujoco model = "test_scene">
    <worldbody>
        <geom type="plane" size="3 3 0.1" rgba="0.6 0.6 0.6 1"/>
        <body name="table" pos="0 0 0.98">
            <geom name="top" type="box" size="0.5 0.5 0.02" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_fl" type="box" size="0.03 0.03 0.48" pos=" 0.45  0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_fr" type="box" size="0.03 0.03 0.48" pos=" 0.45 -0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_bl" type="box" size="0.03 0.03 0.48" pos="-0.45  0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
            <geom name="leg_br" type="box" size="0.03 0.03 0.48" pos="-0.45 -0.45 -0.50" rgba="0.7 0.5 0.3 1"/>
        </body>
        <body name="block" pos="0 0 1.05">
            <freejoint name="block_free"/>
            <geom type="box" size="0.05 0.05 0.05" rgba="0.2 0.5 0.8 1"/>
        </body>
        <body name="sphere" pos="0.2 0 2.0">
            <joint type="free"/>
            <geom type="sphere" size="0.05" rgba="0.8 0.3 0.3 1"/>
        </body>
        <light name="soft_sun" pos="0 0 3" dir="0 0 -1"
            castshadow="true" directional="true"
            ambient="0.1 0.1 0.1" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>
        <camera name="front" pos="1.5 0 1.75" euler="0 70 90"/>
    </worldbody>
    <default>
        <geom friction="1 0.1 0.01"/>
    </default>
</mujoco>
"""
def main():
    model = mujoco.MjModel.from_xml_string(scene)
    data = mujoco.MjData(model)

    block_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "block_free")
    block_qvel = model.jnt_dofadr[block_id]
    data.qvel[block_qvel:block_qvel+6] = [0, 3, 0, 10, 0, 0]  # give the block some initial angular velocity

    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, width = 640, height = 480)

    ### uncomment code below to take static snapshot from front perspective of table

    # renderer.update_scene(data, camera="front")
    # rgb = renderer.render()
    # Image.fromarray(rgb).save("images/static_snapshot.png")

    # open viewer
    print("Opening viewer")
    target_fps = 90
    with viewer.launch_passive(model, data) as view:
        view.cam.lookat[:] = (0.0, 0.0, 1.4)
        view.cam.distance = 2.0
        view.cam.elevation = -35.0
        view.cam.azimuth = 45.0
        while view.is_running():
            mujoco.mj_step(model, data)
            view.sync()
            time.sleep(1 / target_fps)
    print("Viewer closed")

main() # main block implemented to avoid stuck terminal error