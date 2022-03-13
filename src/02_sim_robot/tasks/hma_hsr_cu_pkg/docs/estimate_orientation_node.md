# estimate_orientation_node
This node provides the function to estimate the cutlery and marker orientation using template match.

A hand camera is used for orientation estimation.

## Usage

If `/sm_cu_node/oreint_item/flag` in rosparam is `True`, it is executed.

If `/sm_cu_node/target_obj/id` in rosparam (which stores the ID of the grasped object) is cutlery (`30`, `31`, `32`), then matching is performed with the cutlery template image, and orientation estimation is performed.

In other cases, matching is performed with the marker template image, and orientation estimation is performed.

The estimated result (left or right as seen from the camera) is set to `/sm_cu_node/oreint_item/direction` in rosparam.