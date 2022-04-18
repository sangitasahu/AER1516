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

class JoyDef {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JoyDef
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JoyDef
    let len;
    let data = new JoyDef(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/JoyDef';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e5483257fd0a1adbc2dd471d331e6470';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Message defining constants for the wireless joysticks used by ACL
    
    # buttons
    int64 A=0
    int64 B=1
    int64 X=2
    int64 Y=3
    int64 LB=4
    int64 RB=5
    int64 BACK=6
    int64 START=7
    int64 CENTER=8 # center 'on' button
    int64 LS=9 # left joystick
    int64 RS=10 # right joystick
    int64 LEFT=11
    int64 RIGHT=12
    int64 UP=13
    int64 DOWN=14
    
    # axes
    int64 LEFT_X=0
    int64 LEFT_Y=1
    int64 LT=2
    int64 RIGHT_X=3
    int64 RIGHT_Y=4
    int64 RT=5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JoyDef(null);
    return resolved;
    }
};

// Constants for message
JoyDef.Constants = {
  A: 0,
  B: 1,
  X: 2,
  Y: 3,
  LB: 4,
  RB: 5,
  BACK: 6,
  START: 7,
  CENTER: 8,
  LS: 9,
  RS: 10,
  LEFT: 11,
  RIGHT: 12,
  UP: 13,
  DOWN: 14,
  LEFT_X: 0,
  LEFT_Y: 1,
  LT: 2,
  RIGHT_X: 3,
  RIGHT_Y: 4,
  RT: 5,
}

module.exports = JoyDef;
