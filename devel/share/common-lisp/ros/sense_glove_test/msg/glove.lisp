; Auto-generated. Do not edit!


(cl:in-package sense_glove_test-msg)


;//! \htmlinclude glove.msg.html

(cl:defclass <glove> (roslisp-msg-protocol:ros-message)
  ((finger
    :reader finger
    :initarg :finger
    :type cl:fixnum
    :initform 0)
   (vibration_amplitude
    :reader vibration_amplitude
    :initarg :vibration_amplitude
    :type cl:fixnum
    :initform 0)
   (vibration_duration
    :reader vibration_duration
    :initarg :vibration_duration
    :type cl:fixnum
    :initform 0))
)

(cl:defclass glove (<glove>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <glove>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'glove)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sense_glove_test-msg:<glove> is deprecated: use sense_glove_test-msg:glove instead.")))

(cl:ensure-generic-function 'finger-val :lambda-list '(m))
(cl:defmethod finger-val ((m <glove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sense_glove_test-msg:finger-val is deprecated.  Use sense_glove_test-msg:finger instead.")
  (finger m))

(cl:ensure-generic-function 'vibration_amplitude-val :lambda-list '(m))
(cl:defmethod vibration_amplitude-val ((m <glove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sense_glove_test-msg:vibration_amplitude-val is deprecated.  Use sense_glove_test-msg:vibration_amplitude instead.")
  (vibration_amplitude m))

(cl:ensure-generic-function 'vibration_duration-val :lambda-list '(m))
(cl:defmethod vibration_duration-val ((m <glove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sense_glove_test-msg:vibration_duration-val is deprecated.  Use sense_glove_test-msg:vibration_duration instead.")
  (vibration_duration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <glove>) ostream)
  "Serializes a message object of type '<glove>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vibration_amplitude)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vibration_duration)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <glove>) istream)
  "Deserializes a message object of type '<glove>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'finger)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vibration_amplitude)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'vibration_duration)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<glove>)))
  "Returns string type for a message object of type '<glove>"
  "sense_glove_test/glove")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'glove)))
  "Returns string type for a message object of type 'glove"
  "sense_glove_test/glove")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<glove>)))
  "Returns md5sum for a message object of type '<glove>"
  "ebb0cd83724acf33d4c91ba9d3bf438e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'glove)))
  "Returns md5sum for a message object of type 'glove"
  "ebb0cd83724acf33d4c91ba9d3bf438e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<glove>)))
  "Returns full string definition for message of type '<glove>"
  (cl:format cl:nil "uint8 finger~%uint8 vibration_amplitude~%uint8 vibration_duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'glove)))
  "Returns full string definition for message of type 'glove"
  (cl:format cl:nil "uint8 finger~%uint8 vibration_amplitude~%uint8 vibration_duration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <glove>))
  (cl:+ 0
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <glove>))
  "Converts a ROS message object to a list"
  (cl:list 'glove
    (cl:cons ':finger (finger msg))
    (cl:cons ':vibration_amplitude (vibration_amplitude msg))
    (cl:cons ':vibration_duration (vibration_duration msg))
))
