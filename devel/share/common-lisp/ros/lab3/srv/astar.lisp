; Auto-generated. Do not edit!


(cl:in-package lab3-srv)


;//! \htmlinclude astar-request.msg.html

(cl:defclass <astar-request> (roslisp-msg-protocol:ros-message)
  ((startPose
    :reader startPose
    :initarg :startPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose))
   (endPose
    :reader endPose
    :initarg :endPose
    :type geometry_msgs-msg:Pose
    :initform (cl:make-instance 'geometry_msgs-msg:Pose)))
)

(cl:defclass astar-request (<astar-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <astar-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'astar-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab3-srv:<astar-request> is deprecated: use lab3-srv:astar-request instead.")))

(cl:ensure-generic-function 'startPose-val :lambda-list '(m))
(cl:defmethod startPose-val ((m <astar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-srv:startPose-val is deprecated.  Use lab3-srv:startPose instead.")
  (startPose m))

(cl:ensure-generic-function 'endPose-val :lambda-list '(m))
(cl:defmethod endPose-val ((m <astar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-srv:endPose-val is deprecated.  Use lab3-srv:endPose instead.")
  (endPose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <astar-request>) ostream)
  "Serializes a message object of type '<astar-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'startPose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'endPose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <astar-request>) istream)
  "Deserializes a message object of type '<astar-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'startPose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'endPose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<astar-request>)))
  "Returns string type for a service object of type '<astar-request>"
  "lab3/astarRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'astar-request)))
  "Returns string type for a service object of type 'astar-request"
  "lab3/astarRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<astar-request>)))
  "Returns md5sum for a message object of type '<astar-request>"
  "4ce05851504fd5efe880feba37418384")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'astar-request)))
  "Returns md5sum for a message object of type 'astar-request"
  "4ce05851504fd5efe880feba37418384")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<astar-request>)))
  "Returns full string definition for message of type '<astar-request>"
  (cl:format cl:nil "geometry_msgs/Pose startPose~%geometry_msgs/Pose endPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'astar-request)))
  "Returns full string definition for message of type 'astar-request"
  (cl:format cl:nil "geometry_msgs/Pose startPose~%geometry_msgs/Pose endPose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <astar-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'startPose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'endPose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <astar-request>))
  "Converts a ROS message object to a list"
  (cl:list 'astar-request
    (cl:cons ':startPose (startPose msg))
    (cl:cons ':endPose (endPose msg))
))
;//! \htmlinclude astar-response.msg.html

(cl:defclass <astar-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass astar-response (<astar-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <astar-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'astar-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name lab3-srv:<astar-response> is deprecated: use lab3-srv:astar-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <astar-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader lab3-srv:path-val is deprecated.  Use lab3-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <astar-response>) ostream)
  "Serializes a message object of type '<astar-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <astar-response>) istream)
  "Deserializes a message object of type '<astar-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<astar-response>)))
  "Returns string type for a service object of type '<astar-response>"
  "lab3/astarResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'astar-response)))
  "Returns string type for a service object of type 'astar-response"
  "lab3/astarResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<astar-response>)))
  "Returns md5sum for a message object of type '<astar-response>"
  "4ce05851504fd5efe880feba37418384")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'astar-response)))
  "Returns md5sum for a message object of type 'astar-response"
  "4ce05851504fd5efe880feba37418384")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<astar-response>)))
  "Returns full string definition for message of type '<astar-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'astar-response)))
  "Returns full string definition for message of type 'astar-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <astar-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <astar-response>))
  "Converts a ROS message object to a list"
  (cl:list 'astar-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'astar)))
  'astar-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'astar)))
  'astar-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'astar)))
  "Returns string type for a service object of type '<astar>"
  "lab3/astar")