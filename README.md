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

 * ``enable_mapping``: When set, mapping is disabled. A global NDT will
   be built from the loaded map.

 * ``global_search_size``: The maximum distance between two scans to
   be considered for global loop closure.

 * ``global_search_limit``: The maximum number of scans to be considered
   for global loop closure against a new scan.

 * ``minimum_travel_distance``: Minimum linear travel distance before
   localization update is applied. Applies to both particle filter and
   scan matching based localization. Units: meters.

 * ``minimum_travel_rotation``: Minimum angular travel before
   localization update is applied. Applies to both particle filter and
   scan matching based localization. Units: radians.

 * ``map_file``: If this set, this resource will be loaded as an initial
   map. This works for both continuing to map OR localization. Robot
   must be localized with the initial pose tool.

 * ``max_range``: Maximum distance of laser measurements. Measurements
   beyond this range are discarded. Default is ``-1``, in which case the
   max range will be extracted from the laser scan message.

 * ``occupancy_threshold``: When generating the occupancy grid map, this
   is the threshold between free and occupied space based on how many
   raytraces have hit or passed through a given cell.

 * ``odom_frame``: TF frame_id for the odometry. Usually ``odom``.

 * ``optimization_node_limit``: Minimum number of nodes that must be added
   to the graph between runs of the graph optimizer.

 * ``resolution``: Resolution of the published occupancy grid map. This is
   entirely independent of the underlying resolution of the NDT. Units: meters.

 * ``robot_frame``: TF frame_id for the robot. Usually ``base_link``.

 * ``rolling_depth``: When building a map, this is how many scans to use
   when building the local NDT for scan matching.

 * ``scan_matcher_type``: The plugin name for the scan matcher to use. Default
   is ``ndt_2d::ScanMatcherNDT``.

 * ``transform_timeout``: Max allowable time to wait for transform to become
   available when transforming the laser scan. Units: seconds.

 * ``use_barycenter``: When scan matching, should closest scans be selected
   via the scan pose or the barycenter of the scan points.

 * ``use_particle_filter``: When set, mapping is disabled and a global
   NDT is created. The initial pose tool will initialize localization.

## ScanMatcherNDT Parameters

Each scan matcher uses the following parameters, namespaced into either
``local_scan_matcher`` or ``global_scan_matcher`` namespaces:

 * ``ndt_resolution``: Resolution used for the NDT grid. Every cell of this
   resolution will be represented by a single Gaussian function. Units: meters.

 * ``laser_max_beams``: Maximum number of laser beams to use during scan
   matching. This mirrors the parameter of the same name in AMCL.

 * ``search_angular_resolution``: Angular resolution to use for the scan
   matching search. Units: radians.

 * ``search_angular_size``: Search will be conducted from ``-search_angular_size``
   to ``search_angular_size``, centered around the odometry heading. Units: radians.

 * ``search_linear_resolution``: Linear resolution to use for the scan
   matching search in X/Y dimensions. Units: meters.

 * ``search_linear_size``: Search will be conducted from ``-search_linear_size``
   to ``search_linear_size``, centered around the odometry pose. Units: meters.

## Technical Details

This package implements mapping and localization using the following:

 * The underlying representation for scan matching is the Normal
   Distribution Transform (NDT) as described in [[1]](#1). We do
   not implement the overlapping grids as described in section III
   of the paper. The covariance computation when doing scan matching
   is based on [[2]](#2).

 * The calculation of NDT cell mean and covariances is done in an
   incremental manner modeled on [[3]](#3).

 * The particle filter, motion model, and KLD resampling algorithms
   come from the "Probabilistic Robotics" book [[4]](#4). The
   filter does not include the recovery feature based on tracking
   of average weights as it was unused in every AMCL configuration
   investigated.

## Threading Notes

There are three threads:

 * The rclcpp::spin() thread - this processes the laser scan callback and
   initial pose callbacks. This is the only thread that adds scans to the
   graph. This thread also adds constraints to the graph. This is the only
   thread that changes the prev_X_pose_ variables.

 * The loop closure thread - access graph, adds constraints to the graph.

 * The publish thread - publishes the map and transforms. Accesses graph
   but does not alter the graph. Access prev_X_pose_ variables but does
   not alter them.

## References

<a id="1">[1]</a> Biber, Peter, and Wolfgang Straßer. "The normal distributions transform: A new approach to laser scan matching." Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003)(Cat. No. 03CH37453). Vol. 3. IEEE, 2003.

<a id="2">[2]</a> Olson, Edwin B. "Real-time correlative scan matching." 2009 IEEE International Conference on Robotics and Automation. IEEE, 2009.

<a id="3">[3]</a> Saarinen, Jari, et al. "Normal distributions transform occupancy maps: Application to large-scale online 3D mapping." 2013 IEEE international conference on robotics and automation. IEEE, 2013.

<a id="4">[4]</a> Thrun, Burgard and Fox. "Probabilistic Robotics". MIT Press, 2005.
