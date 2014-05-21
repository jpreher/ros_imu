; Auto-generated. Do not edit!


(cl:in-package leg_pose-msg)


;//! \htmlinclude legPose.msg.html

(cl:defclass <legPose> (roslisp-msg-protocol:ros-message)
  ((shank_earth
    :reader shank_earth
    :initarg :shank_earth
    :type cl:float
    :initform 0.0)
   (thigh_shank
    :reader thigh_shank
    :initarg :thigh_shank
    :type cl:float
    :initform 0.0))
)

(cl:defclass legPose (<legPose>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <legPose>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'legPose)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_pose-msg:<legPose> is deprecated: use leg_pose-msg:legPose instead.")))

(cl:ensure-generic-function 'shank_earth-val :lambda-list '(m))
(cl:defmethod shank_earth-val ((m <legPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_pose-msg:shank_earth-val is deprecated.  Use leg_pose-msg:shank_earth instead.")
  (shank_earth m))

(cl:ensure-generic-function 'thigh_shank-val :lambda-list '(m))
(cl:defmethod thigh_shank-val ((m <legPose>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_pose-msg:thigh_shank-val is deprecated.  Use leg_pose-msg:thigh_shank instead.")
  (thigh_shank m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <legPose>) ostream)
  "Serializes a message object of type '<legPose>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'shank_earth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'thigh_shank))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <legPose>) istream)
  "Deserializes a message object of type '<legPose>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'shank_earth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thigh_shank) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<legPose>)))
  "Returns string type for a message object of type '<legPose>"
  "leg_pose/legPose")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'legPose)))
  "Returns string type for a message object of type 'legPose"
  "leg_pose/legPose")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<legPose>)))
  "Returns md5sum for a message object of type '<legPose>"
  "b95053568fdd0e718d845449ba7102d0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'legPose)))
  "Returns md5sum for a message object of type 'legPose"
  "b95053568fdd0e718d845449ba7102d0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<legPose>)))
  "Returns full string definition for message of type '<legPose>"
  (cl:format cl:nil "float64 shank_earth~%float64 thigh_shank~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'legPose)))
  "Returns full string definition for message of type 'legPose"
  (cl:format cl:nil "float64 shank_earth~%float64 thigh_shank~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <legPose>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <legPose>))
  "Converts a ROS message object to a list"
  (cl:list 'legPose
    (cl:cons ':shank_earth (shank_earth msg))
    (cl:cons ':thigh_shank (thigh_shank msg))
))
