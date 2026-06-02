# Architecture Notes

## 2026-06-02 README usage flow refresh

- Documentation decision: the package README usage section is now organized by operator workflow instead of historical course order: install/build, quick start, feature entrypoints, teleop, SLAM, Nav2, compatibility limits, and troubleshooting.
- Path contract: examples use `~/ros2_ws` and package-relative commands instead of machine-specific absolute paths, so new users can copy commands without adapting `/home/.../ROSExperimenteight` paths.
- Maintenance rule: when adding or removing launch files, update the README feature-entry table in the same change so the public usage surface remains discoverable.

## 2026-04-18 Git divergence handling

- Root cause: local `main` and `origin/main` each had one unique commit, so `git pull` stopped because no pull strategy was configured.
- Operational decision: prefer `rebase` for this repository so local feature work is replayed on top of the latest `origin/main` and the history stays linear.
- Pitfall found: Python cache artifacts under `launch/__pycache__/` and `src/__pycache__/` were versioned. These binary files caused the rebase conflict because Git cannot safely auto-merge `.pyc`.
- Remediation: add a repository-level `.gitignore` for `__pycache__/` and `*.pyc`, then remove tracked cache artifacts from version control to prevent repeated sync failures.

## 2026-05-29 Jazzy water-delivery adaptation

- Target runtime: ROS2 Jazzy on Ubuntu 24.04 with `gz_sim`, `ros_gz`, Nav2, SLAM Toolbox, and `gz_ros2_control`.
- Compatibility decision: do not depend on the old course `wp_map_tools` package for the water-delivery experiment because it is not available in the local Jazzy apt source and is not present in this workspace.
- Replacement: `home_pkg/scripts/waypoint_navi_server.py` keeps the original WaterPlus-style topic contract, subscribing to `/waterplus/navi_waypoint`, sending goals to Nav2 `navigate_to_pose`, and publishing `/waterplus/navi_result`.
- Waypoint storage: use `~/waypoints.yaml` by default. The required names are `kitchen` and `guest`; poses can be stored as simple `x/y/yaw` entries or as ROS-style `position`/`orientation` maps.
- Operational pitfall: every ROS terminal must source `/opt/ros/jazzy/setup.bash` and this workspace's `install/setup.bash`; launch validation in sandboxed environments may need `ROS_LOG_DIR=/tmp/roslogs` because `~/.ros/log` can be read-only.

## 2026-05-29 SLAM startup hardening

- Root cause: the original `wpr_simulation2/launch/slam.launch.py` launched `sync_slam_toolbox_node` directly with only frame names and `use_sim_time`. On ROS2 Jazzy, RViz's SLAM Toolbox panel expects the lifecycle-managed `/slam_toolbox` node and a complete mapper parameter set, so startup could sit in a "waiting for configuration" state.
- Remediation: `slam.launch.py` now includes `slam_toolbox/online_sync_launch.py` and passes `config/slam_toolbox_mapping.yaml`. This keeps node naming, lifecycle configure/activate events, scan topic, frames, solver plugin, and map parameters aligned with the installed Jazzy package.
- Resource decision: SLAM defaults to `spawn_objects:=false`. The world walls and robot sensors still start, but extra furniture/object model creation is skipped by default to avoid WSL/Gazebo stalls during mapping. Full scene mapping remains available with `ros2 launch wpr_simulation2 slam.launch.py spawn_objects:=true`.
- Verification evidence: static regression test `tests/test_slam_launch_contract.py`, `colcon build --packages-up-to home_pkg`, launch argument parsing with `ROS_LOG_DIR=/tmp/roslogs`, and a non-sandbox timed launch smoke test confirmed the new startup contract. The smoke test reached `Configuring`, `Activating`, robot entity creation, `/scan` bridge creation, and SLAM lidar registration before the intentional timeout.

## 2026-05-29 Gazebo GUI visibility hardening

- Root cause: `world.launch.py` previously delegated to `ros_gz_sim/gz_sim.launch.py` with one inline `gz sim -r ...` process. In WSL/VS Code terminals this can keep the Gazebo server alive while no visible GUI window appears, which makes a healthy SLAM run look broken.
- Remediation: `world.launch.py` now starts Gazebo server explicitly with `-s -r` and starts the GUI client separately as a direct launch-tracked `gz sim -g -v 1` process. GUI startup failures are written to the main launch terminal.
- Contract: `launch_rviz:=false` only disables RViz. Gazebo GUI remains enabled by default through `gazebo_gui:=true`; use `gazebo_gui:=false` for true headless runs.
- Verification evidence: `tests/test_slam_launch_contract.py`, `colcon build --packages-up-to home_pkg`, `ros2 launch wpr_simulation2 slam.launch.py --show-args`, and a timed GUI smoke test confirmed that `gz sim -g -v 1` is launched separately from the Gazebo server.

## 2026-05-29 Gazebo GUI version-argument fix

- Root cause: the GUI-only command used `gz sim -g -v 1 --force-version 8`. In ROS2 Jazzy's Gazebo Sim 8 CLI wrapper, that trailing `8` can be parsed as the optional `[file]` argument, producing `Unable to find or download file [8]` and closing the GUI client while the server keeps running.
- Remediation: `world.launch.py` and attach-mode `home.launch.py` now launch the GUI client as `gz sim -g -v 1` and let `gz` auto-select the installed Sim version. The `gazebo_gui_gz_version` launch argument remains only as a deprecated compatibility argument so older command lines do not fail.
- Regression guard: `tests/test_slam_launch_contract.py` asserts that launch-tracked GUI commands no longer contain `--force-version`.

## 2026-05-29 Gazebo GUI WSLg Qt platform fix

- Root cause: the Gazebo GUI process could stay alive without a usable visible window when WSLg exposed both `WAYLAND_DISPLAY` and `DISPLAY`. Unlike RViz, the Gazebo GUI client did not force Qt onto the XCB backend, so Qt could select a Wayland path that produced an invisible or tiny surface in this environment.
- Remediation: both `world.launch.py` and attach-mode `home.launch.py` now start the Gazebo GUI with `QT_QPA_PLATFORM=xcb`, `QT_X11_NO_MITSHM=1`, `MESA_GL_VERSION_OVERRIDE=3.3`, and `OGRE_RTT_MODE=Copy`.
- Regression guard: `tests/test_slam_launch_contract.py` asserts that the Gazebo GUI launch actions carry the WSLg-safe Qt/OpenGL environment.

## 2026-05-29 Gazebo GUI xterm wrapper removal

- Root cause: after the Qt/XCB fix, `xterm -e gz sim -g -v 1` still produced tiny Windows taskbar thumbnails in WSLg. The process tree showed `xterm -> gz sim -g -v 1`, so the visible artifact was the terminal wrapper rather than a reliable Gazebo main window.
- Remediation: the Gazebo GUI client is no longer wrapped in xterm. `world.launch.py` and attach-mode `home.launch.py` run `gz sim -g -v 1` directly under ROS launch.
- Regression guard: `tests/test_slam_launch_contract.py` asserts that the world Gazebo GUI and home attach Gazebo GUI commands do not include `xterm`.

## 2026-05-30 Gazebo GUI window placement fix

- Root cause: `gz sim -g -v 1` successfully created a `Gazebo Sim` X11 window, but WSLg mapped it at a far-right virtual desktop coordinate such as `+2652+453`, so the process was alive while the user-facing monitor showed no window.
- Evidence: `xwininfo -root -tree` reported `"Gazebo Sim": ("gz-sim-gui" "Gazebo GUI") 1000x845 ... +2652+453`; a manual X11 `XMoveResizeWindow`/`XMapRaised` call moved it to `1200x850+80+40` and made it visible.
- Remediation: `wpr_simulation2/src/raise_gazebo_window.py` now recursively finds the `Gazebo Sim` X11 window, moves/raises it, and requests X input focus. `world.launch.py` and attach-mode `home.launch.py` run this helper 3 seconds after the Gazebo GUI client starts.
- Regression guard: `tests/test_slam_launch_contract.py` and `tests/test_home_launch_contract.py` assert that the helper is installed and invoked by both GUI startup paths.

## 2026-05-30 Gazebo service bridge log noise reduction

- Root cause: `RosGzBridge` for `/world/default/set_pose` logged `Creating ROS->GZ service bridge` at INFO level repeatedly. This looked like a launch problem in the terminal even when Gazebo GUI, robot spawn, and controllers were healthy.
- Remediation: `world.launch.py` now runs the service bridge action with `log_level="warn"`, keeping real warnings visible while suppressing the periodic INFO spam.
- Regression guard: `tests/test_slam_launch_contract.py` asserts the quieter log level.

## 2026-05-29 Home water-delivery startup hardening

- Root cause: the original full experiment launch started Gazebo, robot spawning, mechanical arm controllers, Nav2, RViz, object spawns, and person spawns in one burst. On WSL/Gazebo this can look like a frozen terminal even when individual nodes are only waiting for dependencies.
- Map contract: `src/wpr_simulation2/maps/map.yaml` and `map.pgm` are now installed with the package, and `config/nav2_params.yaml` gives `map_server.yaml_filename` a package-relative default: `$(find-pkg-share wpr_simulation2)/maps/map.yaml`. This prevents `map_server` from starting with an empty map and blocking AMCL.
- Startup contract: `home_pkg/launch/home.launch.py` exposes `gazebo_gui`, `spawn_objects`, `spawn_persons`, `start_nav2`, `launch_rviz`, `scene_delay`, `nav2_delay`, and `rviz_delay`. Defaults keep the original user command working, but stage heavy services: scene after 20 s, Nav2 after 30 s, RViz after 90 s.
- Controller contract: `spawn_wpb_mani.launch.py` resolves `ensure_controller_active.py` from the install prefix with `FindPackagePrefix`, so installed launches no longer depend on source-tree helper paths.
- Localization contract: AMCL has a default initial pose near the spawned robot, allowing `map -> base_link` to appear during headless launches without manual RViz input.
- Verification evidence: `tests/test_home_launch_contract.py`, `tests/test_slam_launch_contract.py`, `colcon build --packages-up-to home_pkg`, `ros2 launch home_pkg home.launch.py --show-args`, and a 55 s headless smoke test with `gazebo_gui:=false launch_rviz:=false spawn_objects:=false spawn_persons:=false`. The smoke test reached robot spawn, active `joint_state_broadcaster`, active `manipulator_controller`, map loading from installed package share, AMCL map reception and initial pose, and both Nav2 lifecycle managers reporting `Managed nodes are active`.

## 2026-05-29 Two-stage visual attach for headless home launch

- Root cause: running `ros2 launch home_pkg home.launch.py gazebo_gui:=false launch_rviz:=false spawn_objects:=false spawn_persons:=false` and then running `ros2 launch home_pkg home.launch.py spawn_objects:=true spawn_persons:=true` used to start a second Gazebo server, robot spawn path, Nav2 stack, waypoint server, object publisher, and grab simulator. The duplicate server reset `/clock`, producing repeated TF "jump back in time" warnings and making Gazebo GUI/RViz visibility unreliable.
- Remediation: `home.launch.py` now defaults `attach_existing:=false` so the first terminal starts a deterministic full stack even if stale `/clock` or Gazebo processes are present. Explicit `attach_existing:=true` is required for the second visual/scene attach terminal.
- Attach-mode contract: attach mode starts only the Gazebo GUI client, optional RViz, optional scene/object/person spawns, and optional `fetch_node`. It does not start another Gazebo server, robot, controller, Nav2, waypoint server, object publisher, or grab simulator.
- Operator escape hatches: use `attach_existing:=true` to force attach-only behavior, or keep `attach_existing:=false` to force a complete new stack. `attach_rviz_delay:=3.0` controls the shorter RViz delay used after the headless stack is already warm.

## 2026-05-29 WSL GUI attach robustness

- Root cause: `attach_existing:=auto` originally depended only on ROS/Gazebo topic probes. In WSL/Gazebo, the server process can already be running while ROS graph discovery or Gazebo topic listing is not ready, so a second launch still misclassified the system as "no reusable simulation detected" and restarted the full stack.
- Remediation: existing-simulation detection now has a process fallback: `pgrep -f "gz sim -s .*robocup_home.world"`. This makes the two-stage workflow robust while `/clock` discovery is still warming.
- Visibility decision: attach-mode RViz still launches through `xterm -hold` with fixed geometry, but Gazebo GUI launches directly as `gz sim -g -v 1`. This avoids WSLg exposing a tiny terminal thumbnail instead of a usable Gazebo window.
- Lifecycle decision: attach mode defaults `attach_keep_alive:=true`, adding a lightweight long-running process after the GUI/RViz and scene actions start. Without this, the second launch exits normally as soon as detached GUI wrappers and one-shot model creation processes finish, which looks like a failure even when the windows are still running.

## 2026-05-29 Attach mode made explicit

- Root cause: the terminal-1 command `start_nav2:=false launch_rviz:=false gazebo_gui:=true spawn_objects:=false spawn_persons:=false` still had `gazebo_gui:=true`, so the old `attach_existing:=auto` policy treated stale `/clock` or leftover bridge nodes as proof of an existing simulation and entered attach mode instead of starting Gazebo/robot/controllers.
- Remediation: `attach_existing` now defaults to `false`, and auto mode ignores `gazebo_gui` alone when deciding whether a launch is attach-only work. Gazebo-only startup is therefore a full-stack startup unless the operator explicitly passes `attach_existing:=true`.
- Documentation contract: second-stage visual/scene attach commands must include `attach_existing:=true`.

## 2026-05-29 Gazebo GUI tracking and fetch retry loop

- Root cause: using `setsid -f xterm ...` inside the ROS launch action around the Gazebo GUI client detached that client from launch supervision, so ROS launch could report `gazebo_gui ... finished cleanly` even when the visible Gazebo client was still starting, hidden, or failed outside the launch process. This made GUI startup hard to diagnose from the launch terminal.
- Remediation: `world.launch.py` and attach-mode `home.launch.py` now execute the Gazebo GUI directly as a launch-owned `gz sim -g -v 1` process. A missing DISPLAY or Gazebo GUI failure is visible in the main launch terminal instead of being hidden behind a detached wrapper.
- Root cause: `fetch_node` originally published each phase command only once. A transient subscriber/action-server startup race could lose `/waterplus/navi_waypoint` or `/wpb_home/behavior`, leaving the state machine apparently stuck.
- Remediation: `fetch_node` now exposes `command_retry_ms` and republishes the active stage command every 5 seconds until the expected result arrives. The normal full run is `WAIT -> GOTO_KITCHEN -> GRAB_DRINK -> GOTO_GUEST -> DONE`; users should not stop the node immediately after the first `navi done` because the grab simulation still needs time to finish.

## 2026-05-29 Idempotent waypoint retries

- Root cause: command retry on `/waterplus/navi_waypoint` made `waypoint_navi_server.py` send a fresh Nav2 goal every 5 seconds for the same waypoint. Nav2 treated that as preemption, logged `Received goal preemption request`, and the previous goal returned action status `6`.
- Remediation: `waypoint_navi_server.py` now tracks `active_waypoint_name` plus `active_goal_sequence`. Repeated commands for the same active waypoint are ignored, while stale async Nav2 responses from superseded goals are discarded. This keeps retry resilience without repeatedly canceling the active navigation.
- Runtime note: `wpb_home_planar_move` can briefly warn that `/world/default/set_pose` is not ready while ROS/Gazebo service discovery warms up. The service contract is `/world/default/set_pose` with type `ros_gz_interfaces/srv/SetEntityPose`; persistent warnings after launch warm-up should be diagnosed as a bridge/discovery issue, not a fetch-state-machine issue.

## 2026-05-30 Visible Gazebo startup contract

- Root cause: the home water-delivery launch can start correctly while the operator still sees no usable Gazebo GUI if the command is launched from a terminal whose WSLg/X11 focus, Qt platform, or OpenGL environment is incomplete. In the real non-sandbox WSLg run, the same launch reached robot spawn, active bridges/controllers, and `raise_gazebo_window.py` moved `Gazebo Sim` to `1200x850+80+40`; sandbox failures with `getifaddrs/socket` and `cannot open X display` are not representative of the desktop runtime.
- Remediation: `scripts/start_home_gazebo_gui.sh` is now the preferred terminal-1 entrypoint. It preflights `DISPLAY`, sources ROS Jazzy plus the workspace overlay, pins WSLg-safe GUI variables, and runs the exact `home_pkg home.launch.py start_nav2:=false launch_rviz:=false gazebo_gui:=true spawn_objects:=false spawn_persons:=false` command inside a visible `xterm`.
- Operator contract: use the script for the first Gazebo/robot terminal; keep that xterm open while running RViz, Nav2, and `fetch_node` from later terminals. Default mode intentionally keeps furniture, bottle, and person spawning disabled for WSL startup stability. Use `./scripts/start_home_gazebo_gui.sh --full-scene` when the visible Gazebo scene must include those extra entities from the beginning.

## 2026-05-30 Home Gazebo entrypoint xterm session fix

- Root cause: `scripts/start_home_gazebo_gui.sh` used `exec xterm`, replacing the caller's shell with the GUI terminal. If WSLg/VS Code failed to map that xterm visibly, the operator lost both the original terminal and useful diagnostics even though the ROS launch chain could still start.
- Remediation: the script now uses `setsid -f xterm` for the visible terminal, creates `ROS_LOG_DIR` up front, prints the log path in the original terminal, and exits cleanly without replacing the caller shell.
- Verification evidence: a real GUI probe reached `gazebo_gui`, robot spawn, active controllers, and `gazebo_window_raise`; `tests/test_home_launch_contract.py` guards the detached xterm contract so this entrypoint does not regress back to `exec xterm`.

## 2026-05-30 Nav2 command output without robot motion

- Root cause: `fetch_node` repeatedly logged `Still waiting for navigation to kitchen` because the WaterPlus-style command did reach `waypoint_navi_server`, and Nav2 did accept the `kitchen` goal, but the robot pose stayed fixed at `(-4.0, -0.5)`. Runtime probes showed non-zero `/cmd_vel_nav`, `/cmd_vel_smoothed`, and `/cmd_vel`, followed by `controller_server` aborting with `Failed to make progress`.
- Failing layer: the last bridge, `wpb_home_planar_move`, can freeze if one asynchronous `/world/default/set_pose` request never completes. While `pending_request` remains unresolved, the node keeps publishing the old odom/TF and never integrates later `/cmd_vel`, so Nav2 appears alive but the robot never moves.
- Remediation: `gz_planar_move.py` now has `set_pose_request_timeout` and drops stale Gazebo set-pose futures after the timeout. Late futures are ignored, preventing an old response from clearing a newer in-flight request. This keeps simulated odom/TF moving even when one ros_gz service response stalls.

## 2026-05-30 Minimal Nav2 stack for delivery task

- Root cause: a real run of terminal 4 showed `fetch_node` receiving repeated `navi failed` results because `waypoint_navi_server` could not find the Nav2 `navigate_to_pose` action server. Runtime evidence showed localization active and several navigation nodes launched, but `bt_navigator` never became active because Jazzy's default `nav2_bringup/navigation_launch.py` lifecycle list blocked while configuring optional `route_server`.
- Architecture decision: the water-delivery experiment does not use Nav2 route planning extensions, collision monitor polygons, or docking behavior. Starting them creates unnecessary lifecycle failure points before the required `bt_navigator` action server is available.
- Remediation: `home_pkg/launch/nav2_minimal.launch.py` now starts localization through Nav2's stock localization launch, then starts only the delivery-critical navigation nodes: `controller_server`, `smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`, and `velocity_smoother`. `home.launch.py` and the manual terminal-3 command use this minimal launch instead of `nav2_bringup bringup_launch.py`.
- Follow-up root cause: after the minimal stack activated, `bt_navigator` accepted the `kitchen` goal and `controller_server` published non-zero `/cmd_vel_nav`, but `/cmd_vel` stayed silent. In the stock Jazzy bringup, `collision_monitor` forwards `cmd_vel_smoothed` to `cmd_vel`; removing `collision_monitor` requires the minimal launch to make that output remap explicitly.
- Remediation: `nav2_minimal.launch.py` remaps `velocity_smoother` input `cmd_vel` to `cmd_vel_nav` and output `cmd_vel_smoothed` to `cmd_vel`, so `wpb_home_planar_move` receives the velocity command it listens for.

## 2026-05-30 Delivery grasp fallback and guest handoff

- Root cause: the simulated grasp stage could stall when `/wpb_home/objects_3d` arrived late, arrived empty, or contained invalid coordinates. That left `grab_object_sim` waiting forever in the kitchen, so `fetch_node` never received `grab done` and the downstream `guest` navigation never started.
- Remediation: `grab_object_sim.cpp` now validates incoming object arrays, picks the most plausible tabletop object instead of blindly reading index 0, and falls back to a known safe simulated drink pose after a short wait if no usable object pose arrives.
- Guardrail: the alignment and forward motion are clamped so a bad detection cannot drive the base at an unreasonable speed or sleep for a negative duration.
- Operator note: if terminal 4 still stalls in `STEP_GRAB_DRINK` after a rebuild, restart the full Gazebo stack so the new `grab_object_sim` binary is the one actually running.

## 2026-05-30 Kitchen fallback pose sync

- Root cause: after Nav2 reached `kitchen`, the grasp stage could still steer toward a pre-modification drink location because `objects_publisher.cpp` was taking fallback bottle poses that were already authored in the robot/base frame and then transforming them again through `/odom`. That double transform turned a valid kitchen-relative pose into a stale world-relative offset, so `grab_object_sim` aligned against the wrong bottle location and appeared to drift back.
- Remediation: keep the delivery fallback constants in the robot frame end-to-end. `objects_publisher.cpp` now publishes fallback bottles directly at `(1.00, -0.15)` and `(1.00, 0.15)` without an odom-based retransform, and `grab_object_sim.cpp` continues to bias its single-object fallback around `x=1.00`.
- Operational constraint: if the kitchen table or bottles move again, update the fallback constants in both `objects_publisher.cpp` and `grab_object_sim.cpp` together; changing only one side reintroduces an apparent "drift back to old pose" during the grab step.

## 2026-05-30 Kitchen pickup handoff without reverse motion

- Root cause: the simulated grasp node still performed a post-grasp `STEP_BACKWARD` motion after lifting the drink. That base motion fought the new delivery handoff, so the robot appeared to drift away from the kitchen before Nav2 could resume the `guest` delivery leg.
- Remediation: remove the reverse-drive stage from `src/wpr_simulation2/src/grab_object_sim.cpp`. After `STEP_OBJ_UP`, the node now parks the base, transitions directly to `STEP_DONE`, and publishes `grab done` so `fetch_node` can immediately issue the `guest` waypoint.
- Guardrail: the regression test now rejects any reintroduction of `STEP_BACKWARD` or the negative reverse velocity command in the grab node.

## 2026-05-30 Gazebo bottle attachment during simulated grasp

- Root cause: the grasp node only closed the gripper joints. The kitchen drink model stayed a free Gazebo rigid body, so the operator saw a closed hand but no visible pickup.
- Remediation: `grab_object_sim.cpp` now tracks the robot pose from `/odom`, selects the kitchen bottle entity, and calls `/world/default/set_pose` to keep that model attached in the robot frame once the grasp closes. The bottle is then updated through the guest delivery leg so the visual pickup survives after `grab done`.
- Guardrail: the regression test now checks for `/odom`, `SetEntityPose`, and the bottle-follow offsets so this does not regress back to a cosmetic-only grasp.

## 2026-05-30 Static fallback grasp alignment guard

- Root cause: the kitchen fallback drink poses are static robot-frame coordinates. `grab_object_sim.cpp` treated them like live point-cloud detections and kept driving `linear.y` toward the fallback bottle's fixed lateral offset. Because that synthetic `y` value does not shrink when the base moves, the align loop could command sideways motion indefinitely and push the robot out of the room.
- Remediation: fallback object names now carry a `sim_fallback_` prefix. The grasp node recognizes that source and suppresses lateral closed-loop chasing for static fallback poses while still allowing front/back alignment. Real point-cloud detections can still drive lateral correction.
- Guardrail: the align phase now has a hard timeout that publishes zero `/cmd_vel`, stops object publishing, and proceeds with a bounded grasp rather than letting any stale or non-converging detection drive the base forever.

## 2026-05-30 Gripper-centered bottle display

- Root cause: the first Gazebo bottle attachment used a coarse base-frame offset (`base + 0.34 m forward + 0.08 m lateral`) rather than the WPB manipulator geometry. The model was technically following the robot, but the bottle appeared away from the physical gripper in Gazebo.
- Remediation: `grab_object_sim.cpp` now derives the held bottle pose from `wpb_home_mani.model` geometry: the manipulator lift base, elbow offset, finger reach, gripper lateral center, and commanded lift height. The bottle base is placed just below that computed gripper center before calling `/world/default/set_pose`.
- Guardrail: `tests/test_home_launch_contract.py` rejects the old held-object offsets and asserts the model-derived gripper constants remain wired into `ComposeAttachedWorldPose()`.
