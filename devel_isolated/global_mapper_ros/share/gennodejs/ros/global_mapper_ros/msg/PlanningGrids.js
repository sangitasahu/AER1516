// Auto-generated. Do not edit!

// (in-package global_mapper_ros.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class PlanningGrids {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.projected_goal = null;
      this.origin = null;
      this.grid_dimensions = null;
      this.resolution = null;
      this.occupancy_data = null;
      this.distance_data = null;
      this.cost_data = null;
      this.occupied_threshold = null;
      this.dmax = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('projected_goal')) {
        this.projected_goal = initObj.projected_goal
      }
      else {
        this.projected_goal = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('origin')) {
        this.origin = initObj.origin
      }
      else {
        this.origin = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('grid_dimensions')) {
        this.grid_dimensions = initObj.grid_dimensions
      }
      else {
        this.grid_dimensions = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('resolution')) {
        this.resolution = initObj.resolution
      }
      else {
        this.resolution = 0.0;
      }
      if (initObj.hasOwnProperty('occupancy_data')) {
        this.occupancy_data = initObj.occupancy_data
      }
      else {
        this.occupancy_data = [];
      }
      if (initObj.hasOwnProperty('distance_data')) {
        this.distance_data = initObj.distance_data
      }
      else {
        this.distance_data = [];
      }
      if (initObj.hasOwnProperty('cost_data')) {
        this.cost_data = initObj.cost_data
      }
      else {
        this.cost_data = [];
      }
      if (initObj.hasOwnProperty('occupied_threshold')) {
        this.occupied_threshold = initObj.occupied_threshold
      }
      else {
        this.occupied_threshold = 0.0;
      }
      if (initObj.hasOwnProperty('dmax')) {
        this.dmax = initObj.dmax
      }
      else {
        this.dmax = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PlanningGrids
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [projected_goal] has the right length
    if (obj.projected_goal.length !== 3) {
      throw new Error('Unable to serialize array field projected_goal - length must be 3')
    }
    // Serialize message field [projected_goal]
    bufferOffset = _arraySerializer.float64(obj.projected_goal, buffer, bufferOffset, 3);
    // Check that the constant length array field [origin] has the right length
    if (obj.origin.length !== 3) {
      throw new Error('Unable to serialize array field origin - length must be 3')
    }
    // Serialize message field [origin]
    bufferOffset = _arraySerializer.float64(obj.origin, buffer, bufferOffset, 3);
    // Check that the constant length array field [grid_dimensions] has the right length
    if (obj.grid_dimensions.length !== 3) {
      throw new Error('Unable to serialize array field grid_dimensions - length must be 3')
    }
    // Serialize message field [grid_dimensions]
    bufferOffset = _arraySerializer.int32(obj.grid_dimensions, buffer, bufferOffset, 3);
    // Serialize message field [resolution]
    bufferOffset = _serializer.float64(obj.resolution, buffer, bufferOffset);
    // Serialize message field [occupancy_data]
    bufferOffset = _arraySerializer.float32(obj.occupancy_data, buffer, bufferOffset, null);
    // Serialize message field [distance_data]
    bufferOffset = _arraySerializer.int32(obj.distance_data, buffer, bufferOffset, null);
    // Serialize message field [cost_data]
    bufferOffset = _arraySerializer.int32(obj.cost_data, buffer, bufferOffset, null);
    // Serialize message field [occupied_threshold]
    bufferOffset = _serializer.float32(obj.occupied_threshold, buffer, bufferOffset);
    // Serialize message field [dmax]
    bufferOffset = _serializer.int32(obj.dmax, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PlanningGrids
    let len;
    let data = new PlanningGrids(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [projected_goal]
    data.projected_goal = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [origin]
    data.origin = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [grid_dimensions]
    data.grid_dimensions = _arrayDeserializer.int32(buffer, bufferOffset, 3)
    // Deserialize message field [resolution]
    data.resolution = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [occupancy_data]
    data.occupancy_data = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [distance_data]
    data.distance_data = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [cost_data]
    data.cost_data = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [occupied_threshold]
    data.occupied_threshold = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [dmax]
    data.dmax = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 4 * object.occupancy_data.length;
    length += 4 * object.distance_data.length;
    length += 4 * object.cost_data.length;
    return length + 88;
  }

  static datatype() {
    // Returns string type for a message object
    return 'global_mapper_ros/PlanningGrids';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '86653f2401f7e451293296585b426430';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    float64[3] projected_goal  # current position of projected goal within the grid
    float64[3] origin  # center of map in global frame [m]
    int32[3] grid_dimensions  # size of map [voxels]
    float64 resolution  # voxel size [m]
    
    float32[] occupancy_data
    int32[] distance_data
    int32[] cost_data
    
    float32 occupied_threshold #occupancy values above this are considered 'occupied'
    int32 dmax  #max squared distance reported in distance_grid
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PlanningGrids(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.projected_goal !== undefined) {
      resolved.projected_goal = msg.projected_goal;
    }
    else {
      resolved.projected_goal = new Array(3).fill(0)
    }

    if (msg.origin !== undefined) {
      resolved.origin = msg.origin;
    }
    else {
      resolved.origin = new Array(3).fill(0)
    }

    if (msg.grid_dimensions !== undefined) {
      resolved.grid_dimensions = msg.grid_dimensions;
    }
    else {
      resolved.grid_dimensions = new Array(3).fill(0)
    }

    if (msg.resolution !== undefined) {
      resolved.resolution = msg.resolution;
    }
    else {
      resolved.resolution = 0.0
    }

    if (msg.occupancy_data !== undefined) {
      resolved.occupancy_data = msg.occupancy_data;
    }
    else {
      resolved.occupancy_data = []
    }

    if (msg.distance_data !== undefined) {
      resolved.distance_data = msg.distance_data;
    }
    else {
      resolved.distance_data = []
    }

    if (msg.cost_data !== undefined) {
      resolved.cost_data = msg.cost_data;
    }
    else {
      resolved.cost_data = []
    }

    if (msg.occupied_threshold !== undefined) {
      resolved.occupied_threshold = msg.occupied_threshold;
    }
    else {
      resolved.occupied_threshold = 0.0
    }

    if (msg.dmax !== undefined) {
      resolved.dmax = msg.dmax;
    }
    else {
      resolved.dmax = 0
    }

    return resolved;
    }
};

module.exports = PlanningGrids;
