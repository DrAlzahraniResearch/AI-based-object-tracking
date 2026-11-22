"""Gazebo integration (milestone 6).

* `models`    SDF for the viewer's vehicle markers and camera rig, camera poses
* `transport` gz-transport Python bindings (Harmonic, Ionic or Jetty) and `gz` CLI fallbacks, the sim clock
* `view`      draws `swarm_sim.viz.scene.Frame`s into a running world: vehicle models, intent/prediction markers
* `record`    headless video from a camera sensor, stepping a paused world frame by frame
* `live`      a twin-log tap that publishes the twin on a gz topic, and the matching subscriber
* `launch`    starting and stopping `gz sim`, PX4's Gazebo environment

Tier B in Gazebo (PX4 SITL + x500) is `swarm_sim.twin.px4_gz.Px4GzPlant`.
"""
