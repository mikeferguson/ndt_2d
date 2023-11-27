# ndt_2d

This package implements the Normal Distribution Transform (NDT) for mapping
and localization.

## Use Cases

 * Mapping: default parameters should work out of the box. The save_map.py
   script can be used to save the NDT map data for later localization.
   The regular nav2_map_server can be used to save the map needed for
   navigation, or it can be re-generated at runtime from the NDT map data.

 * Localization via Particle Filter: set the ``map_file`` parameter to the
   full filename of your saved NDT map data, set ``use_particle_filter``
   to ``true``. The particle filter supports many of the same odometry and
   filter parameters as AMCL (e.g. ``odom_alpha1``, ``min_particles``);

 * Localization via Scan Matching: set the ``map_file`` parameter to the
   full filename of your saved NDT map data, make sure ``use_particle_filter``
   is set to ``false``.

## Parameter Details

 * ``map_file``: If this set, this resource will be loaded as an initial
   map. This works for both continuing to map OR localization. Robot
   must be localized with the initial pose tool.
 * ``enable_mapping``: When set, mapping is disabled. A global NDT will
   be built from the loaded map.
 * ``use_particle_filter``: When set, mapping is disabled and a global
   NDT is created. The initial pose tool will initialize localization.
