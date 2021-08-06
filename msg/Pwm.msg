#Standard metadata for higher-level flow data types
#sequence ID: consecutively increasing ID
# uint32 seq
# string frame_id
#Two-integer timestamp that is expressed as:
# * stamp.secs: seconds (stamp_secs) since epoch
# * stamp.nsecs: nanoseconds since stamp_secs
# time-handling sugar is provided by the client library
# time stamp
#Frame this data is associated with
Header header
uint32 pwm_val
uint32 dir