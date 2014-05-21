; Auto-generated. Do not edit!


(cl:in-package tau_under-msg)


;//! \htmlinclude tau_under_msg.msg.html

(cl:defclass <tau_under_msg> (roslisp-msg-protocol:ros-message)
  ((left_shank_earth
    :reader left_shank_earth
    :initarg :left_shank_earth
    :type cl:float
    :initform 0.0)
   (left_thigh_shank
    :reader left_thigh_shank
    :initarg :left_thigh_shank
    :type cl:float
    :initform 0.0)
   (left_hip_thigh
    :reader left_hip_thigh
    :initarg :left_hip_thigh
    :type cl:float
    :initform 0.0)
   (right_shank_earth
    :reader right_shank_earth
    :initarg :right_shank_earth
    :type cl:float
    :initform 0.0)
   (right_thigh_shank
    :reader right_thigh_shank
    :initarg :right_thigh_shank
    :type cl:float
    :initform 0.0)
   (right_hip_thigh
    :reader right_hip_thigh
    :initarg :right_hip_thigh
    :type cl:float
    :initform 0.0)
   (tau
    :reader tau
    :initarg :tau
    :type cl:float
    :initform 0.0)
   (right_is_stance
    :reader right_is_stance
    :initarg :right_is_stance
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass tau_under_msg (<tau_under_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <tau_under_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'tau_under_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tau_under-msg:<tau_under_msg> is deprecated: use tau_under-msg:tau_under_msg instead.")))

(cl:ensure-generic-function 'left_shank_earth-val :lambda-list '(m))
(cl:defmethod left_shank_earth-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:left_shank_earth-val is deprecated.  Use tau_under-msg:left_shank_earth instead.")
  (left_shank_earth m))

(cl:ensure-generic-function 'left_thigh_shank-val :lambda-list '(m))
(cl:defmethod left_thigh_shank-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:left_thigh_shank-val is deprecated.  Use tau_under-msg:left_thigh_shank instead.")
  (left_thigh_shank m))

(cl:ensure-generic-function 'left_hip_thigh-val :lambda-list '(m))
(cl:defmethod left_hip_thigh-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:left_hip_thigh-val is deprecated.  Use tau_under-msg:left_hip_thigh instead.")
  (left_hip_thigh m))

(cl:ensure-generic-function 'right_shank_earth-val :lambda-list '(m))
(cl:defmethod right_shank_earth-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:right_shank_earth-val is deprecated.  Use tau_under-msg:right_shank_earth instead.")
  (right_shank_earth m))

(cl:ensure-generic-function 'right_thigh_shank-val :lambda-list '(m))
(cl:defmethod right_thigh_shank-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:right_thigh_shank-val is deprecated.  Use tau_under-msg:right_thigh_shank instead.")
  (right_thigh_shank m))

(cl:ensure-generic-function 'right_hip_thigh-val :lambda-list '(m))
(cl:defmethod right_hip_thigh-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:right_hip_thigh-val is deprecated.  Use tau_under-msg:right_hip_thigh instead.")
  (right_hip_thigh m))

(cl:ensure-generic-function 'tau-val :lambda-list '(m))
(cl:defmethod tau-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:tau-val is deprecated.  Use tau_under-msg:tau instead.")
  (tau m))

(cl:ensure-generic-function 'right_is_stance-val :lambda-list '(m))
(cl:defmethod right_is_stance-val ((m <tau_under_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tau_under-msg:right_is_stance-val is deprecated.  Use tau_under-msg:right_is_stance instead.")
  (right_is_stance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <tau_under_msg>) ostream)
  "Serializes a message object of type '<tau_under_msg>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_shank_earth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_thigh_shank))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'left_hip_thigh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_shank_earth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_thigh_shank))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'right_hip_thigh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tau))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'right_is_stance) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <tau_under_msg>) istream)
  "Deserializes a message object of type '<tau_under_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_shank_earth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_thigh_shank) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'left_hip_thigh) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_shank_earth) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_thigh_shank) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'right_hip_thigh) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tau) (roslisp-utils:decode-double-float-bits bits)))
    (cl:setf (cl:slot-value msg 'right_is_stance) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<tau_under_msg>)))
  "Returns string type for a message object of type '<tau_under_msg>"
  "tau_under/tau_under_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'tau_under_msg)))
  "Returns string type for a message object of type 'tau_under_msg"
  "tau_under/tau_under_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<tau_under_msg>)))
  "Returns md5sum for a message object of type '<tau_under_msg>"
  "6be40ba2e8dfc4bfad571b61abbe7647")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'tau_under_msg)))
  "Returns md5sum for a message object of type 'tau_under_msg"
  "6be40ba2e8dfc4bfad571b61abbe7647")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<tau_under_msg>)))
  "Returns full string definition for message of type '<tau_under_msg>"
  (cl:format cl:nil "float64 left_shank_earth~%float64 left_thigh_shank~%float64 left_hip_thigh~%float64 right_shank_earth~%float64 right_thigh_shank~%float64 right_hip_thigh~%float64 tau~%bool right_is_stance~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'tau_under_msg)))
  "Returns full string definition for message of type 'tau_under_msg"
  (cl:format cl:nil "float64 left_shank_earth~%float64 left_thigh_shank~%float64 left_hip_thigh~%float64 right_shank_earth~%float64 right_thigh_shank~%float64 right_hip_thigh~%float64 tau~%bool right_is_stance~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <tau_under_msg>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <tau_under_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'tau_under_msg
    (cl:cons ':left_shank_earth (left_shank_earth msg))
    (cl:cons ':left_thigh_shank (left_thigh_shank msg))
    (cl:cons ':left_hip_thigh (left_hip_thigh msg))
    (cl:cons ':right_shank_earth (right_shank_earth msg))
    (cl:cons ':right_thigh_shank (right_thigh_shank msg))
    (cl:cons ':right_hip_thigh (right_hip_thigh msg))
    (cl:cons ':tau (tau msg))
    (cl:cons ':right_is_stance (right_is_stance msg))
))
