// Auto-generated. Do not edit!

// (in-package convex_decomposer.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Polyhedron = require('./Polyhedron.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CvxDecomp {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.polyhedra = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('polyhedra')) {
        this.polyhedra = initObj.polyhedra
      }
      else {
        this.polyhedra = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CvxDecomp
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [polyhedra]
    // Serialize the length for message field [polyhedra]
    bufferOffset = _serializer.uint32(obj.polyhedra.length, buffer, bufferOffset);
    obj.polyhedra.forEach((val) => {
      bufferOffset = Polyhedron.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CvxDecomp
    let len;
    let data = new CvxDecomp(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [polyhedra]
    // Deserialize array length for message field [polyhedra]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.polyhedra = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.polyhedra[i] = Polyhedron.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.polyhedra.forEach((val) => {
      length += Polyhedron.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'convex_decomposer/CvxDecomp';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa0b44be5161dc2f552ca9c3ef45157c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    convex_decomposer/Polyhedron[] polyhedra
    
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
    
    ================================================================================
    MSG: convex_decomposer/Polyhedron
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
    const resolved = new CvxDecomp(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.polyhedra !== undefined) {
      resolved.polyhedra = new Array(msg.polyhedra.length);
      for (let i = 0; i < resolved.polyhedra.length; ++i) {
        resolved.polyhedra[i] = Polyhedron.Resolve(msg.polyhedra[i]);
      }
    }
    else {
      resolved.polyhedra = []
    }

    return resolved;
    }
};

module.exports = CvxDecomp;
