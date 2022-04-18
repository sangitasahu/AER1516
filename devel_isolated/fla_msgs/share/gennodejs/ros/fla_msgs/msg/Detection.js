// Auto-generated. Do not edit!

// (in-package fla_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Detection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.class_id = null;
      this.class_name = null;
      this.confidence = null;
      this.x_min = null;
      this.y_min = null;
      this.x_max = null;
      this.y_max = null;
      this.object_id = null;
      this.x_pos = null;
      this.y_pos = null;
      this.z_pos = null;
    }
    else {
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = 0;
      }
      if (initObj.hasOwnProperty('class_name')) {
        this.class_name = initObj.class_name
      }
      else {
        this.class_name = '';
      }
      if (initObj.hasOwnProperty('confidence')) {
        this.confidence = initObj.confidence
      }
      else {
        this.confidence = 0.0;
      }
      if (initObj.hasOwnProperty('x_min')) {
        this.x_min = initObj.x_min
      }
      else {
        this.x_min = 0.0;
      }
      if (initObj.hasOwnProperty('y_min')) {
        this.y_min = initObj.y_min
      }
      else {
        this.y_min = 0.0;
      }
      if (initObj.hasOwnProperty('x_max')) {
        this.x_max = initObj.x_max
      }
      else {
        this.x_max = 0.0;
      }
      if (initObj.hasOwnProperty('y_max')) {
        this.y_max = initObj.y_max
      }
      else {
        this.y_max = 0.0;
      }
      if (initObj.hasOwnProperty('object_id')) {
        this.object_id = initObj.object_id
      }
      else {
        this.object_id = 0;
      }
      if (initObj.hasOwnProperty('x_pos')) {
        this.x_pos = initObj.x_pos
      }
      else {
        this.x_pos = 0.0;
      }
      if (initObj.hasOwnProperty('y_pos')) {
        this.y_pos = initObj.y_pos
      }
      else {
        this.y_pos = 0.0;
      }
      if (initObj.hasOwnProperty('z_pos')) {
        this.z_pos = initObj.z_pos
      }
      else {
        this.z_pos = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Detection
    // Serialize message field [class_id]
    bufferOffset = _serializer.uint32(obj.class_id, buffer, bufferOffset);
    // Serialize message field [class_name]
    bufferOffset = _serializer.string(obj.class_name, buffer, bufferOffset);
    // Serialize message field [confidence]
    bufferOffset = _serializer.float32(obj.confidence, buffer, bufferOffset);
    // Serialize message field [x_min]
    bufferOffset = _serializer.float32(obj.x_min, buffer, bufferOffset);
    // Serialize message field [y_min]
    bufferOffset = _serializer.float32(obj.y_min, buffer, bufferOffset);
    // Serialize message field [x_max]
    bufferOffset = _serializer.float32(obj.x_max, buffer, bufferOffset);
    // Serialize message field [y_max]
    bufferOffset = _serializer.float32(obj.y_max, buffer, bufferOffset);
    // Serialize message field [object_id]
    bufferOffset = _serializer.uint32(obj.object_id, buffer, bufferOffset);
    // Serialize message field [x_pos]
    bufferOffset = _serializer.float32(obj.x_pos, buffer, bufferOffset);
    // Serialize message field [y_pos]
    bufferOffset = _serializer.float32(obj.y_pos, buffer, bufferOffset);
    // Serialize message field [z_pos]
    bufferOffset = _serializer.float32(obj.z_pos, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Detection
    let len;
    let data = new Detection(null);
    // Deserialize message field [class_id]
    data.class_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [class_name]
    data.class_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [confidence]
    data.confidence = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_min]
    data.x_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_min]
    data.y_min = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [x_max]
    data.x_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_max]
    data.y_max = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [object_id]
    data.object_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [x_pos]
    data.x_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [y_pos]
    data.y_pos = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [z_pos]
    data.z_pos = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.class_name.length;
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/Detection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f4c53395944a41874a6ab9783c6e93b6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 class_id
    string class_name
    float32 confidence
    
    # (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates
    float32 x_min 
    float32 y_min
    float32 x_max
    float32 y_max
    
    # ground truth unique identifier and position of the detected object (Simulation only) 
    uint32 object_id
    float32 x_pos
    float32 y_pos
    float32 z_pos
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Detection(null);
    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = 0
    }

    if (msg.class_name !== undefined) {
      resolved.class_name = msg.class_name;
    }
    else {
      resolved.class_name = ''
    }

    if (msg.confidence !== undefined) {
      resolved.confidence = msg.confidence;
    }
    else {
      resolved.confidence = 0.0
    }

    if (msg.x_min !== undefined) {
      resolved.x_min = msg.x_min;
    }
    else {
      resolved.x_min = 0.0
    }

    if (msg.y_min !== undefined) {
      resolved.y_min = msg.y_min;
    }
    else {
      resolved.y_min = 0.0
    }

    if (msg.x_max !== undefined) {
      resolved.x_max = msg.x_max;
    }
    else {
      resolved.x_max = 0.0
    }

    if (msg.y_max !== undefined) {
      resolved.y_max = msg.y_max;
    }
    else {
      resolved.y_max = 0.0
    }

    if (msg.object_id !== undefined) {
      resolved.object_id = msg.object_id;
    }
    else {
      resolved.object_id = 0
    }

    if (msg.x_pos !== undefined) {
      resolved.x_pos = msg.x_pos;
    }
    else {
      resolved.x_pos = 0.0
    }

    if (msg.y_pos !== undefined) {
      resolved.y_pos = msg.y_pos;
    }
    else {
      resolved.y_pos = 0.0
    }

    if (msg.z_pos !== undefined) {
      resolved.z_pos = msg.z_pos;
    }
    else {
      resolved.z_pos = 0.0
    }

    return resolved;
    }
};

module.exports = Detection;
