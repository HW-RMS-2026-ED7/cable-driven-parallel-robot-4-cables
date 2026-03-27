from kinematics import (
    Z as FRAME_Z,
    Z_EFF,
    END_EFFECTOR_MASS,
    S,
    E_W,
    E_L,
    E_H,
    SPOOL_RADIUS,
)

# --- 1. Robot Parameters ---

# MuJoCo uses half-sizes for box geoms
hx, hy, hz = E_W / 2, E_L / 2, E_H / 2

# --- 2. Dynamic XML Generation ---
mjcf_xml = f"""
<mujoco model="4_cable_robot">
    <compiler angle="radian"/>

    <option gravity="0 0 -9.81"/>

    <default>
        <tendon width="0.005" rgba="1 1 1 1"/>
        <position kp="5000" kv="50"/> 
    </default>

    <worldbody>
        <light pos="0 0 5" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
        <geom type="plane" size="3 3 0.1" rgba="0.2 0.2 0.2 1"/>

        <site name="anchor_0" pos="{S / 2} {S / 2} {FRAME_Z}" size="0.02" rgba="1 0 0 1"/>
        <site name="anchor_1" pos="{-S / 2} {S / 2} {FRAME_Z}" size="0.02" rgba="1 0 0 1"/>
        <site name="anchor_2" pos="{-S / 2} {-S / 2} {FRAME_Z}" size="0.02" rgba="1 0 0 1"/>
        <site name="anchor_3" pos="{S / 2} {-S / 2} {FRAME_Z}" size="0.02" rgba="1 0 0 1"/>

        <body name="end_effector" pos="0 0 {Z_EFF}">
            <freejoint/>
            <geom type="box" size="{hx} {hy} {hz}" mass="{END_EFFECTOR_MASS}" rgba="0 0 1 1"/>

            <site name="platform_0" pos="{hx} {hy} 0" size="0.015" rgba="0 1 0 1"/>
            <site name="platform_1" pos="{-hx} {hy} 0" size="0.015" rgba="0 1 0 1"/>
            <site name="platform_2" pos="{-hx} {-hy} 0" size="0.015" rgba="0 1 0 1"/>
            <site name="platform_3" pos="{hx} {-hy} 0" size="0.015" rgba="0 1 0 1"/>
        </body>
    </worldbody>

    <tendon>
        <spatial name="cable_0"><site site="anchor_0"/><site site="platform_0"/></spatial>
        <spatial name="cable_1"><site site="anchor_1"/><site site="platform_1"/></spatial>
        <spatial name="cable_2"><site site="anchor_2"/><site site="platform_2"/></spatial>
        <spatial name="cable_3"><site site="anchor_3"/><site site="platform_3"/></spatial>
    </tendon>

    <actuator>
        <position name="motor_0" tendon="cable_0" gear="{SPOOL_RADIUS}"/>
        <position name="motor_1" tendon="cable_1" gear="{SPOOL_RADIUS}"/>
        <position name="motor_2" tendon="cable_2" gear="{SPOOL_RADIUS}"/>
        <position name="motor_3" tendon="cable_3" gear="{SPOOL_RADIUS}"/>
    </actuator>
</mujoco>
"""

# --- 3. Save the XML to a file ---
with open("cable_robot.xml", "w") as f:
    f.write(mjcf_xml)
