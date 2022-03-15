# mapping_object_server
<img src="resources/mapping_object.jpg"/><br>
This action server provides the function of mapping objects in a designated area for strategic tidy-up.

## Interfaces
### Parameters
**~area_data_path (str)**

Specify the path to the JSON file containing information on the area to be mapped.

The JSON file should be written in the following format.
```json
{
    "PLACE_NAME":{
        "min":{
            "x": value (float),
            "y": value (float),
            "z": value (float)
        },
        "max":{
            "x": value (float),
            "y": value (float),
            "z": value (float)
        }
    }
}
```
An example JSON file can be found [here](../io/mapping_area/area.json).

Defaults to `"hma_common_pkg/io/mapping_area/area.json"`.

**~select_mode (int)**

Select the method to calculate one final recognition result from multiple recognition results.

Two methods are available. 
- When the input is `0`, the most frequent ID is the final recognition result. 
- When the input is `1`, the highest score's ID is the final recognition result.

Defaults to `1`.

**~frame (str)**

The frame name used for mapping.

Defaults to `"map"`.

### Published topics
**~area ([visualization_msgs/Marker.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))**

Publish markers for visualization of the specified area.

**~points ([visualization_msgs/Marker.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))**

Publish markers for visualization indicating the location of mapping objects.

**~text ([visualization_msgs/Marker.msg](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))**

Publish markers for visualization indicating the name of mapping objects.

### Action servers
**mapping_object ([hma_common_action/MappingObjectAction](https://hibikino-musashi-home.github.io/hma_wrs_sim_ws/src/01_common/hma_common/hma_common_action/action/MappingObject.html))**

This action server provides various functions such as adding objects, performing mappings, and referencing objects.