# Mobile Manipulator Collision Geometry Workflow

This note documents the command-line workflow for maintaining self-collision link pairs and optional sphere-only collision URDFs for the mobile manipulator examples.

## Collision Pair Files

OCS2 self-collision config uses `selfCollision.collisionLinkPairs` in `task.info`. A link pair expands to all collision geometry object pairs attached to those two links, so the runtime cost depends on both the number of link pairs and the number of collision objects per link.

Use the helper to validate a pair file against a URDF:

```bash
python3 ocs2_robotic_examples/ocs2_mobile_manipulator/scripts/ocs2_collision_pairs.py validate \
  --urdf /home/asterich/code/cpp/ocs2/src/ocs2_robotic_assets/resources/mobile_manipulator/franka/urdf/panda.urdf \
  --pairs ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/collision_pairs_moveit.info
```

For MoveIt-generated disabled collision matrices, use the headless exporter:

```bash
ocs2_robotic_examples/ocs2_mobile_manipulator/scripts/export_moveit_collision_pairs.sh \
  --urdf <robot.urdf> \
  --srdf <robot.srdf> \
  --output <collision_pairs.info>
```

This wraps `ros2 run moveit_setup_assistant collisions_updater` and does not launch the setup assistant GUI.

## Sphere Collision URDFs

The mobile manipulator interface supports an optional second URDF for self-collision geometry:

```info
selfCollision
{
  collisionUrdfFile "panda_collision_spheres.urdf"
}
```

The main `urdfFile` still defines dynamics and kinematics. The collision URDF must keep the same kinematic model after applying the same `modelType` and `removeJoints` settings: same `nq/nv`, joint names, and frame names.

For URDFs whose collision geometry is already primitive spheres, cylinders, or boxes, convert cylinders/boxes to sphere-only collision geometry:

```bash
python3 ocs2_robotic_examples/ocs2_mobile_manipulator/scripts/ocs2_spherize_primitive_urdf.py \
  --input /home/asterich/code/cpp/ocs2/src/ocs2_robotic_assets/resources/mobile_manipulator/franka/urdf/panda.urdf \
  --output ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/panda_collision_spheres.urdf
```

The primitive converter drops duplicate spheres per link. When converting cylinders or boxes, it snaps each generated coordinate component back to the
source collision origin component if the difference is below `--snap-tolerance`. This removes floating-point rotation residue such as `1e-18`
without clipping legitimate small coordinates by absolute value. The default snap tolerance is `1e-14`.

For URDFs whose collision geometry uses meshes, use the foam wrapper:

```bash
ocs2_robotic_examples/ocs2_mobile_manipulator/scripts/foam_batch_spherize.sh \
  --urdf <robot.urdf> \
  --output-dir <output_dir>
```

## Validation

Run structural validation after generating a sphere collision URDF:

```bash
python3 ocs2_robotic_examples/ocs2_mobile_manipulator/scripts/ocs2_validate_sphere_urdf.py \
  --mesh-urdf /home/asterich/code/cpp/ocs2/src/ocs2_robotic_assets/resources/mobile_manipulator/franka/urdf/panda.urdf \
  --sphere-urdf ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/panda_collision_spheres.urdf \
  --pairs ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/collision_pairs_moveit.info
```

Compare sampled distances through the OCS2 Pinocchio geometry path:

```bash
LD_LIBRARY_PATH=/opt/openrobots/lib:/home/asterich/code/cpp/ocs2/install/ocs2_mobile_manipulator/lib:/home/asterich/code/cpp/ocs2/install/ocs2_self_collision/lib:/home/asterich/code/cpp/ocs2/install/ocs2_pinocchio_interface/lib:/home/asterich/code/cpp/ocs2/install/ocs2_core/lib \
/home/asterich/code/cpp/ocs2/install/ocs2_mobile_manipulator/lib/ocs2_mobile_manipulator/ocs2_compare_collision_urdfs \
  --mesh-urdf /home/asterich/code/cpp/ocs2/src/ocs2_robotic_assets/resources/mobile_manipulator/franka/urdf/panda.urdf \
  --sphere-urdf ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/panda_collision_spheres.urdf \
  --pairs ocs2_robotic_examples/ocs2_mobile_manipulator/config/franka/collision_pairs_moveit.info \
  --samples 50 \
  --seed 2 \
  --tolerance 0.12
```

## Ridgeback UR5 Pair Sets

The Ridgeback UR5 config keeps the lightweight arm self-collision set inline in `config/ridgeback_ur5/task.info`. Additional candidate sets are split into separate files:

- `config/ridgeback_ur5/collision_pairs_arm_self.info`
- `config/ridgeback_ur5/collision_pairs_arm_base.info`
- `config/ridgeback_ur5/collision_pairs_arm_wheels.info`
- `config/ridgeback_ur5/collision_pairs.info` for the combined candidate set

Start with `arm_self` when using the main mesh collision URDF. Add base and wheel pairs only when the runtime cost is acceptable.
