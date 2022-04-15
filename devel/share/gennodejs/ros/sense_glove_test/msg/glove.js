// Auto-generated. Do not edit!

// (in-package sense_glove_test.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class glove {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.finger = null;
      this.vibration_amplitude = null;
      this.vibration_duration = null;
    }
    else {
      if (initObj.hasOwnProperty('finger')) {
        this.finger = initObj.finger
      }
      else {
        this.finger = 0;
      }
      if (initObj.hasOwnProperty('vibration_amplitude')) {
        this.vibration_amplitude = initObj.vibration_amplitude
      }
      else {
        this.vibration_amplitude = 0;
      }
      if (initObj.hasOwnProperty('vibration_duration')) {
        this.vibration_duration = initObj.vibration_duration
      }
      else {
        this.vibration_duration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type glove
    // Serialize message field [finger]
    bufferOffset = _serializer.uint8(obj.finger, buffer, bufferOffset);
    // Serialize message field [vibration_amplitude]
    bufferOffset = _serializer.uint8(obj.vibration_amplitude, buffer, bufferOffset);
    // Serialize message field [vibration_duration]
    bufferOffset = _serializer.uint8(obj.vibration_duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type glove
    let len;
    let data = new glove(null);
    // Deserialize message field [finger]
    data.finger = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vibration_amplitude]
    data.vibration_amplitude = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [vibration_duration]
    data.vibration_duration = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 3;
  }

  static datatype() {
    // Returns string type for a message object
    return 'sense_glove_test/glove';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ebb0cd83724acf33d4c91ba9d3bf438e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 finger
    uint8 vibration_amplitude
    uint8 vibration_duration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new glove(null);
    if (msg.finger !== undefined) {
      resolved.finger = msg.finger;
    }
    else {
      resolved.finger = 0
    }

    if (msg.vibration_amplitude !== undefined) {
      resolved.vibration_amplitude = msg.vibration_amplitude;
    }
    else {
      resolved.vibration_amplitude = 0
    }

    if (msg.vibration_duration !== undefined) {
      resolved.vibration_duration = msg.vibration_duration;
    }
    else {
      resolved.vibration_duration = 0
    }

    return resolved;
    }
};

module.exports = glove;
