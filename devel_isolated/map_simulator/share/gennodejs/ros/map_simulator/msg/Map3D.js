// Auto-generated. Do not edit!

// (in-package map_simulator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Map3D {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nodeCoordinates = null;
      this.occupiedStatus = null;
      this.nodeDistances = null;
    }
    else {
      if (initObj.hasOwnProperty('nodeCoordinates')) {
        this.nodeCoordinates = initObj.nodeCoordinates
      }
      else {
        this.nodeCoordinates = [];
      }
      if (initObj.hasOwnProperty('occupiedStatus')) {
        this.occupiedStatus = initObj.occupiedStatus
      }
      else {
        this.occupiedStatus = [];
      }
      if (initObj.hasOwnProperty('nodeDistances')) {
        this.nodeDistances = initObj.nodeDistances
      }
      else {
        this.nodeDistances = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Map3D
    // Serialize message field [nodeCoordinates]
    // Serialize the length for message field [nodeCoordinates]
    bufferOffset = _serializer.uint32(obj.nodeCoordinates.length, buffer, bufferOffset);
    obj.nodeCoordinates.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Vector3.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [occupiedStatus]
    bufferOffset = _arraySerializer.bool(obj.occupiedStatus, buffer, bufferOffset, null);
    // Serialize message field [nodeDistances]
    bufferOffset = _arraySerializer.uint16(obj.nodeDistances, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Map3D
    let len;
    let data = new Map3D(null);
    // Deserialize message field [nodeCoordinates]
    // Deserialize array length for message field [nodeCoordinates]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodeCoordinates = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodeCoordinates[i] = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [occupiedStatus]
    data.occupiedStatus = _arrayDeserializer.bool(buffer, bufferOffset, null)
    // Deserialize message field [nodeDistances]
    data.nodeDistances = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.nodeCoordinates.length;
    length += object.occupiedStatus.length;
    length += 2 * object.nodeDistances.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'map_simulator/Map3D';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ad6c8afc9d0105744f261886c76d5da8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Vector3[] nodeCoordinates
    bool[] occupiedStatus
    uint16[] nodeDistances
    
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Map3D(null);
    if (msg.nodeCoordinates !== undefined) {
      resolved.nodeCoordinates = new Array(msg.nodeCoordinates.length);
      for (let i = 0; i < resolved.nodeCoordinates.length; ++i) {
        resolved.nodeCoordinates[i] = geometry_msgs.msg.Vector3.Resolve(msg.nodeCoordinates[i]);
      }
    }
    else {
      resolved.nodeCoordinates = []
    }

    if (msg.occupiedStatus !== undefined) {
      resolved.occupiedStatus = msg.occupiedStatus;
    }
    else {
      resolved.occupiedStatus = []
    }

    if (msg.nodeDistances !== undefined) {
      resolved.nodeDistances = msg.nodeDistances;
    }
    else {
      resolved.nodeDistances = []
    }

    return resolved;
    }
};

module.exports = Map3D;
