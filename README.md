# PX4 MPC - Interfacing PX4 with Model Predictive Control
This package contains an MPC integrated with with [PX4 Autopilot](https://px4.io/) and [ROS 2](https://ros.org/).

The MPC uses the [acados framework](https://github.com/acados/acados)

![px4-mpc](https://github.com/user-attachments/assets/6713b8e6-815f-42fe-b3a0-51708d3416e5)

## Citing PX4-MPC
If you find this package useful in an academic context, please consider citing the paper

- Roque, Pedro, Sujet Phodapol, Elias Krantz, Jaeyoung Lim, Joris Verhagen, Frank Jiang, David Dorner, Roland Siegwart, Ivan Stenius, Gunnar Tibert, Huina Mao, Jana Tumova, Christer Fuglesang, Dimos V. Dimarogonas. "Towards Open-Source and Modular Space Systems with ATMOS." arXiv preprint arXiv:2501.16973 (2025).
. [[preprint](https://arxiv.org/abs/2501.16973)]

```
@article{roque2025towards,
  title={Towards Open-Source and Modular Space Systems with ATMOS},
  author={Roque, Pedro and Phodapol, Sujet and Krantz, Elias and Lim, Jaeyoung and Verhagen, Joris and Jiang, Frank and Dorner, David and Siegwart, Roland and Stenius, Ivan and Tibert, Gunnar and others},
  journal={arXiv preprint arXiv:2501.16973},
  year={2025}
}
```

## Setup
The MPC formulation uses acados. In order to install acados, follow the following [instructions](https://docs.acados.org/installation/). After building `acados` remember to install the python interface as described [here](https://docs.acados.org/python_interface/index.html).

acados also needs two environment variables set at **runtime** (not just build time), or solvers will
fail to load with an error like `OSError: libqpOASES_e.so: cannot open shared object file`:
```bash
export ACADOS_SOURCE_DIR=/path/to/acados
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_SOURCE_DIR/lib
```
Add these to your shell profile (e.g. `~/.bashrc`) so every terminal that runs an MPC node or script has them.

To build the code, do the following steps:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/PX4/px4_msgs.git  # PX4 messages for communication with PX4 DDS
git clone https://github.com/Jaeyoung-Lim/px4-offboard.git  # Rviz interface and setpoint generation
git clone https://github.com/DISCOWER/px4-mpc.git  # this package
cd ..
colcon build --packages-up-to px4_mpc
source install/setup.bash  # use source install/setup.zsh for zsh users 
```

## Running MPC with PX4 SITL
In order to run the SITL(Software-In-The-Loop) simulation, the PX4 simulation environment and ROS2 needs to be setup.
For instructions, follow the [documentation](https://docs.px4.io/main/en/ros/ros2_comm.html)

Before proceding, start the DDS interface of PX4 by running
```bash
micro-xrce-dds-agent udp4 --port 8888
```


### Quadrotor Example

On a new terminal, navigate to your PX4-Autopilot directory and run the following command to start the PX4 SITL with Gazebo:
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
On another terminal, start PX4-MPC
```bash
ros2 launch px4_mpc mpc_quadrotor_launch.py 
```

Now, check the QGC or Headless setup in section **QGC Setup or Headless (no-QGC) Setup** at the bottom of this guide.

Then, in the same terminal where you started the PX4 SITL, you can control the quadrotor using the following commands:
- To arm the vehicle:
```bash
commander arm
```
- To follow the MPC setpoints:
```bash
commander mode offboard
```

### Spacecraft Example
First, make sure that you have followed the instructions in the [ATMOS guide](https://atmos.discower.io/pages/Simulation/). ATMOS MPC node also requires vehicle angular velocity data. Please modify the file `PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics-yaml` and uncomment lines 56-57 to enable:
```yaml
  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity
```

Then, on your PX4-Autopilot directory, run the following command to start the PX4 SITL with Gazebo:
```bash
cd ~/PX4-Autopilot
make px4_sitl_spacecraft gz_atmos
```

On another terminal, run
```bash
ros2 launch px4_mpc mpc_spacecraft_launch.py mode:=wrench setpoint_from_rviz:=False
```

Then, in the same terminal where you started the PX4 SITL, you can control ATMOS using the following commands:
- To arm the vehicle:
```bash
commander arm
```
- To follow the MPC setpoints:
```bash
commander mode offboard
```

At this point, the vehicle should start following the desired setpoints.

#### Notes:
The `mpc_spacecraft_launch.py` file includes optional arguments:
- **mode**: Control mode (wrench by default). Options: wrench, rate, direct_allocation and _offset free_. For offset free MPC, please install the package [OpenMPC](https://github.com/mikaelj-kth-se/OpenMPC).  
- **namespace**: Spacecraft namespace ('' by default).  
- **setpoint_from_rviz**: Use RViz for setpoints (True by default).

**Example with no namespace:**
```bash
ros2 launch px4_mpc mpc_spacecraft_launch.py mode:=wrench setpoint_from_rviz:=False
```

**Example with namespace:**
For this example to work, make sure you have run the PX4 SITL with the same namespace. Here goes an example
```bash
PX4_UXRCE_DDS_NS=pop make px4_sitl_spacecraft gz_atmos
```

## Fixed-Wing Minimum-Sink Glide Analysis

`FixedWingEnergyMPC` (`px4_mpc/px4_mpc/controllers/fixedwing_energy_mpc.py`) is a zero-throttle
glide controller whose cost is the aircraft's specific energy rate (`edot = g*hdot + V*Vdot`),
built on `FixedWingEnergyModel` (`px4_mpc/px4_mpc/models/fixedwing_energy_model.py`), which adds
an explicit lift/drag polar to the reduced fixed-wing dynamics. With throttle locked to zero,
minimizing `-edot` is equivalent to minimizing sink rate, and the controller converges to the
classical minimum-sink trim airspeed regardless of the aircraft's initial speed or attitude.

To reproduce the sink-polar plot (zero-throttle trim curve, the min-sink/best-glide points, and
two closed-loop MPC runs converging to the trim from arbitrary off-trim, banked initial states):

```bash
cd ~/ros2_ws/src/px4-mpc/px4_mpc/px4_mpc/test
python3 plot_min_sink_polar.py
```

This requires `acados` (see [Setup](#setup) above) plus `numpy`, `scipy`, and `matplotlib`. It opens
the plot in a window by default; pass `--out <path>` to save a PNG instead, or `--steps <n>` to
change the closed-loop simulation length (default 500 steps at `Ts=0.05s`, i.e. 25s per run).

## QGC Setup or Headless (no-QGC) Setup

You can either use QGroundControl (QGC) to visualize the vehicle state and send commands, or run the simulation headless without QGC. By default, PX4 will expect RC inputs and a QGC connection before allowing arming and mode switching. These are the recommended settings for using the vehicle in real deployment. To this end, do the followings steps:

1. Open QGroundControl.
2. Click the QGroundControl icon on the top left corner and open "Application Settings".
3. Under the "Fly View" tab, scroll down to "Virtual Joystics" and enable them by clicking the slider to the right of "Enabled".
4. Make sure that auto-center throttle is disabled.
5. Click "Exit Application Settings" on the top left corner to return to the Fly-view screen.

**Important:** Before arming the vehicle, make sure that the throttle (left vertical joystick) is at the lowest position.


**Warning: do these at your own risk. We recommend using the QGC interface to ensure hardware and software homogeneity.** If you prefer to run the simulation without QGC, you can disable the RC check and QGC connection requirement by adding the following parameters to the PX4 SITL command:
```bash
pxh> param set COM_RC_IN_MODE 1
pxh> param set NAV_DLL_ACT 0
pxh> param set COM_ARM_WO_GPS 1
pxh> param set CBRK_USB_CHK 197848
pxh> param set COM_ARMABLE 1
```
Then, restart the PX4 SITL to apply the changes. After rebooting, you should be able to arm and switch vehicle modes.