// Auto-generated. Do not edit!

// (in-package fla_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let NodeStatus = require('./NodeStatus.js');

//-----------------------------------------------------------

class NodeList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.stamp = null;
      this.nodelog = null;
    }
    else {
      if (initObj.hasOwnProperty('stamp')) {
        this.stamp = initObj.stamp
      }
      else {
        this.stamp = {secs: 0, nsecs: 0};
      }
      if (initObj.hasOwnProperty('nodelog')) {
        this.nodelog = initObj.nodelog
      }
      else {
        this.nodelog = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NodeList
    // Serialize message field [stamp]
    bufferOffset = _serializer.time(obj.stamp, buffer, bufferOffset);
    // Serialize message field [nodelog]
    // Serialize the length for message field [nodelog]
    bufferOffset = _serializer.uint32(obj.nodelog.length, buffer, bufferOffset);
    obj.nodelog.forEach((val) => {
      bufferOffset = NodeStatus.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NodeList
    let len;
    let data = new NodeList(null);
    // Deserialize message field [stamp]
    data.stamp = _deserializer.time(buffer, bufferOffset);
    // Deserialize message field [nodelog]
    // Deserialize array length for message field [nodelog]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.nodelog = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.nodelog[i] = NodeStatus.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 21 * object.nodelog.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/NodeList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ef5328a9c9c28a94588f0215d3bce4ca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    time stamp
    NodeStatus[] nodelog
    
    ================================================================================
    MSG: fla_msgs/NodeStatus
    uint8 INACTIVE=1
    uint8 STARTING=2
    uint8 INIT=3
    uint8 READY=4
    uint8 ALARM=5
    uint8 FAIL=6
    uint8 SILENT=7
    uint32 id
    uint32 pid
    uint8 status
    int32 arg
    float64 load
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NodeList(null);
    if (msg.stamp !== undefined) {
      resolved.stamp = msg.stamp;
    }
    else {
      resolved.stamp = {secs: 0, nsecs: 0}
    }

    if (msg.nodelog !== undefined) {
      resolved.nodelog = new Array(msg.nodelog.length);
      for (let i = 0; i < resolved.nodelog.length; ++i) {
        resolved.nodelog[i] = NodeStatus.Resolve(msg.nodelog[i]);
      }
    }
    else {
      resolved.nodelog = []
    }

    return resolved;
    }
};

module.exports = NodeList;
