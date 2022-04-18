// Auto-generated. Do not edit!

// (in-package convex_decomposer.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let shape_msgs = _finder('shape_msgs');

//-----------------------------------------------------------

class Polyhedron {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.planes = null;
    }
    else {
      if (initObj.hasOwnProperty('planes')) {
        this.planes = initObj.planes
      }
      else {
        this.planes = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Polyhedron
    // Serialize message field [planes]
    // Serialize the length for message field [planes]
    bufferOffset = _serializer.uint32(obj.planes.length, buffer, bufferOffset);
    obj.planes.forEach((val) => {
      bufferOffset = shape_msgs.msg.Plane.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Polyhedron
    let len;
    let data = new Polyhedron(null);
    // Deserialize message field [planes]
    // Deserialize array length for message field [planes]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.planes = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.planes[i] = shape_msgs.msg.Plane.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 32 * object.planes.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'convex_decomposer/Polyhedron';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '18bc595cd5fca2d6b49e654a7e19a442';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    shape_msgs/Plane[] planes
    
    ================================================================================
    MSG: shape_msgs/Plane
    # Representation of a plane, using the plane equation ax + by + cz + d = 0
    
    # a := coef[0]
    # b := coef[1]
    # c := coef[2]
    # d := coef[3]
    
    float64[4] coef
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Polyhedron(null);
    if (msg.planes !== undefined) {
      resolved.planes = new Array(msg.planes.length);
      for (let i = 0; i < resolved.planes.length; ++i) {
        resolved.planes[i] = shape_msgs.msg.Plane.Resolve(msg.planes[i]);
      }
    }
    else {
      resolved.planes = []
    }

    return resolved;
    }
};

module.exports = Polyhedron;
