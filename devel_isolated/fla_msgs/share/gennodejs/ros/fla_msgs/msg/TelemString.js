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

class TelemString {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nodeid = null;
      this.text = null;
    }
    else {
      if (initObj.hasOwnProperty('nodeid')) {
        this.nodeid = initObj.nodeid
      }
      else {
        this.nodeid = 0;
      }
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TelemString
    // Serialize message field [nodeid]
    bufferOffset = _serializer.uint32(obj.nodeid, buffer, bufferOffset);
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TelemString
    let len;
    let data = new TelemString(null);
    // Deserialize message field [nodeid]
    data.nodeid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.text.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/TelemString';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '90c5a36c3a388c55ce89c05cc31213e3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint32 nodeid
    string text
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TelemString(null);
    if (msg.nodeid !== undefined) {
      resolved.nodeid = msg.nodeid;
    }
    else {
      resolved.nodeid = 0
    }

    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    return resolved;
    }
};

module.exports = TelemString;
