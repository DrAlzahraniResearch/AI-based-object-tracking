"""Visualization: one scene model of a twin run, several viewers.

`scene` turns a twin run (finished or live) into frames: vehicle estimates with conformance, plant truth,
predictions, the intent boxes in force, and events. Viewers only draw frames:

* Gazebo (scripts/gz_view.py, swarm_sim/gz/): 3D world, vehicle models, intent markers, headless video
* Foxglove (scripts/foxglove_export.py, swarm_sim/viz/foxglove.py): 3D scene + plots from one MCAP file
* Rerun (scripts/twin_view.py): scrubbable 3D timeline
"""
