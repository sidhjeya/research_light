; Auto-generated. Do not edit!


(cl:in-package test-msg)


;//! \htmlinclude hi.msg.html

(cl:defclass <hi> (roslisp-msg-protocol:ros-message)
  ((real
    :reader real
    :initarg :real
    :type cl:float
    :initform 0.0)
   (imaginary
    :reader imaginary
    :initarg :imaginary
    :type cl:float
    :initform 0.0))
)

(cl:defclass hi (<hi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name test-msg:<hi> is deprecated: use test-msg:hi instead.")))

(cl:ensure-generic-function 'real-val :lambda-list '(m))
(cl:defmethod real-val ((m <hi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:real-val is deprecated.  Use test-msg:real instead.")
  (real m))

(cl:ensure-generic-function 'imaginary-val :lambda-list '(m))
(cl:defmethod imaginary-val ((m <hi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader test-msg:imaginary-val is deprecated.  Use test-msg:imaginary instead.")
  (imaginary m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hi>) ostream)
  "Serializes a message object of type '<hi>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'real))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'imaginary))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hi>) istream)
  "Deserializes a message object of type '<hi>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'real) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'imaginary) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hi>)))
  "Returns string type for a message object of type '<hi>"
  "test/hi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hi)))
  "Returns string type for a message object of type 'hi"
  "test/hi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hi>)))
  "Returns md5sum for a message object of type '<hi>"
  "54da470dccf15d60bd273ab751e1c0a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hi)))
  "Returns md5sum for a message object of type 'hi"
  "54da470dccf15d60bd273ab751e1c0a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hi>)))
  "Returns full string definition for message of type '<hi>"
  (cl:format cl:nil "float32 real~%float32 imaginary~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hi)))
  "Returns full string definition for message of type 'hi"
  (cl:format cl:nil "float32 real~%float32 imaginary~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hi>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hi>))
  "Converts a ROS message object to a list"
  (cl:list 'hi
    (cl:cons ':real (real msg))
    (cl:cons ':imaginary (imaginary msg))
))
