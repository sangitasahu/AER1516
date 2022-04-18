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

class FlightCommand {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.command = null;
    }
    else {
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlightCommand
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlightCommand
    let len;
    let data = new FlightCommand(null);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/FlightCommand';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3fa8261f600a67acfe8d66a49672fd7b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 CMD_TAKEOFF=0
    uint8 CMD_GO=1
    uint8 CMD_LAND=2
    uint8 CMD_HOVER=3
    uint8 CMD_INIT=4
    uint8 CMD_KILL=5
    uint8 CMD_START_SEARCH=6
    uint8 command
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlightCommand(null);
    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    return resolved;
    }
};

// Constants for message
FlightCommand.Constants = {
  CMD_TAKEOFF: 0,
  CMD_GO: 1,
  CMD_LAND: 2,
  CMD_HOVER: 3,
  CMD_INIT: 4,
  CMD_KILL: 5,
  CMD_START_SEARCH: 6,
}

module.exports = FlightCommand;
