
ros2 topic pub /range sensor_msgs/msg/Range "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, radiation_type: 1, field_of_view: 30, min_range: 5, max_range: 20, range: 5 }"  --once

ros2 topic pub /polygon geometry_msgs/msg/PolygonStamped "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, polygon: { points: [{ x: 1, y: 2, z: 3 },{ x: -4, y: 5, z: -6 },{ x: -7, y: 8, z: 9 }] } }"  --once

ros2 topic pub /fluidpressure sensor_msgs/msg/FluidPressure "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, fluid_pressure: 1000, variance: 0 }"  --once
ros2 topic pub /illuminance sensor_msgs/msg/Illuminance "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, illuminance: 10000, variance: 0 }"  --once
ros2 topic pub /relativehumidity sensor_msgs/msg/RelativeHumidity "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, relative_humidity: 1000, variance: 0 }"  --once
ros2 topic pub /temperature sensor_msgs/msg/Temperature "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, temperature: 100, variance: 0 }"  --once

ros2 topic pub /accelstamped geometry_msgs/msg/AccelStamped "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, accel: { linear: { x: 1, y: 2, z: 2 }, angular: { x: 1, y: 1, z: 1 } } }"  --once
ros2 topic pub /twiststamped geometry_msgs/msg/TwistStamped "{header: { stamp: { sec: 1, nanosec: 1 }, frame_id: 'map' }, twist: { linear: { x: 0.5, y: 1, z: 2 }, angular: { x: 0.5, y: 1.5, z: 2 } } }"  --once
