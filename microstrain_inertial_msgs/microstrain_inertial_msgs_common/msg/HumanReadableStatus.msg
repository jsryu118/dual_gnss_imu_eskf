# Convenience message to contain some useful device information
#
# This is a combination of several different messages to allow for an easy view of the device status.
# For more detailed information on the device status, see the messages mentioned in the field definitions below.
#
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
# NOTE: This is meant as a useful tool for checking general status of the device by a human.
#       It is STRONGLY encouraged to not use this to actually control real logic.
#       For that, we would recommend looking at the other messages mentioned in this one
#       as well as the uncertainty for measurements
# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# Header containing the time at which this message was populated
#   header.frame_id will contain the frame ID of the IMU
std_msgs/Header header

# Device information. This information will be read when the node first activates.
# To force an update of this information (shouldn't change during runtime), call the mip/base/get_device_information service
microstrain_inertial_msgs/MipBaseDeviceInfo device_info

# GNSS state. See GNSS_STATE_* for possible values
# This value is computed from many different fields from microstrain_inertial_msgs/MipGnssFixInfo.msg and microstrain_inertial_msgs/MipFilterGnssPositionAidingStatus.msg
string gnss_state

# Dual antenna fix type. See DUAL_ANTENNA_FIX_TYPE_* enums for possble values
# This is identical to fix_type in microstrain_inertial_msgs/MipFilterGnssDualAntennaStatus.msg
string dual_antenna_fix_type

# Device-specific filter state. See filter_state_* enums for possible values
# This is identical to filter_state in microstrain_inertial_msgs/MipFilterStatus.msg represented as a string
string filter_state

# String version of the status_flags field of the MipFilterStatus message. Only active statuses will be in this list
# This is a list of all of the status_flags_* that are true in microstrain_inertial_msgs/MipFilterStatus.msg filtered to only include relevant flags for this type of device
# See STATIS_FLAG_* for possible values that could be in this array
string[] status_flags

# Possible value for any field if a devicedoes not support it
string UNSUPPORTED = "Unsupported"

# Possible values for gnss_state
string GNSS_STATE_NO_FIX    = "No Fix"
string GNSS_STATE_3D_FIX    = "3D Fix"
string GNSS_STATE_SBAS      = "SBAS"
string GNSS_STATE_RTK_FLOAT = "RTK Float"
string GNSS_STATE_RTK_FIXED = "RTK Fixed"

# Possible values for dual_antenna_fix_type
string DUAL_ANTENNA_FIX_TYPE_NONE  = "None"
string DUAL_ANTENNA_FIX_TYPE_FLOAT = "Dual Antenna Float"
string DUAL_ANTENNA_FIX_TYPE_FIXED = "Dual Antenna Fixed"

# Possible values for filter_state when the connected device is a philo device (GX5, CX5, CV5, etc)
string FILTER_STATE_GX5_STARTUP            = "Startup"
string FILTER_STATE_GX5_INIT               = "Init"
string FILTER_STATE_GX5_RUN_SOLUTION_VALID = "Solution Valid"
string FILTER_STATE_GX5_RUN_SOLUTION_ERROR = "Solution Error"

# Possible values for filter_state when the connected device is a prospect device (GQ7, CV7, etc)
string FILTER_STATE_GQ7_INIT      = "Init"
string FILTER_STATE_GQ7_VERT_GYRO = "Vertical Gyro"
string FILTER_STATE_GQ7_AHRS      = "AHRS"
string FILTER_STATE_GQ7_FULL_NAV  = "Full Nav"

# Possible values to be stored in status_flags when the connected device is a philo device (GX5, CX5, CV5, etc)
string STATUS_FLAGS_GX5_INIT_NO_ATTITUDE                           = "Init No Attitude"
string STATUS_FLAGS_GX5_INIT_NO_POSITION_VELOCITY                  = "Init No Position / Velocity"
string STATUS_FLAGS_GX5_RUN_IMU_UNAVAILABLE                        = "IMU Unavailable"
string STATUS_FLAGS_GX5_RUN_GPS_UNAVAILABLE                        = "GPS Unavailable"
string STATUS_FLAGS_GX5_RUN_MATRIX_SINGULARITY                     = "Matrix Singularity"
string STATUS_FLAGS_GX5_RUN_POSITION_COVARIANCE_WARNING            = "Position Covariance Warning"
string STATUS_FLAGS_GX5_RUN_VELOCITY_COVARIANCE_WARNING            = "Velocity Covariance Warning"
string STATUS_FLAGS_GX5_RUN_ATTITUDE_COVARIANCE_WARNING            = "Attitude Covariance Warning"
string STATUS_FLAGS_GX5_RUN_NAN_IN_SOLUTION_WARNING                = "NaN in Solution"
string STATUS_FLAGS_GX5_RUN_GYRO_BIAS_EST_HIGH_WARNING             = "Gyroscope Bias Estimate High"
string STATUS_FLAGS_GX5_RUN_ACCEL_BIAS_EST_HIGH_WARNING            = "Accelerometer Bias Estimate High"
string STATUS_FLAGS_GX5_RUN_GYRO_SCALE_FACTOR_EST_HIGH_WARNING     = "Gyroscope Scale Factor Estimate High"
string STATUS_FLAGS_GX5_RUN_ACCEL_SCALE_FACTOR_EST_HIGH_WARNING    = "Accelerometer Scal Factor Estimate High"
string STATUS_FLAGS_GX5_RUN_MAG_BIAS_EST_HIGH_WARNING              = "Magnetometer Bias Estimate High"
string STATUS_FLAGS_GX5_RUN_ANT_OFFSET_CORRECTION_EST_HIGH_WARNING = "Antenna Offset Correction Estimate High"
string STATUS_FLAGS_GX5_RUN_MAG_HARD_IRON_EST_HIGH_WARNING         = "Magnetometer Hard Iron Estimate High"
string STATUS_FLAGS_GX5_RUN_MAG_SOFT_IRON_EST_HIGH_WARNING         = "Magnetometer Soft Iron Estimate High"

# Possible values to be stored in status_flags when the connected device is a prospect device (GQ7, CV7, etc)
string STATUS_FLAGS_GQ7_FILTER_CONDITION_STABLE     = "Stable"
string STATUS_FLAGS_GQ7_FILTER_CONDITION_CONVERGING = "Converging"
string STATUS_FLAGS_GQ7_FILTER_CONDITION_UNSTABLE   = "Unstable"
string STATUS_FLAGS_GQ7_ROLL_PITCH_WARNING          = "Roll / Pitch Warning"
string STATUS_FLAGS_GQ7_HEADING_WARNING             = "Heading Warning"
string STATUS_FLAGS_GQ7_POSITION_WARNING            = "Position Warning"
string STATUS_FLAGS_GQ7_VELOCITY_WARNING            = "Velocity Warning"
string STATUS_FLAGS_GQ7_IMU_BIAS_WARNING            = "IMU Bias Warning"
string STATUS_FLAGS_GQ7_GNSS_CLK_WARNING            = "GNSS Clock Warning"
string STATUS_FLAGS_GQ7_ANTENNA_LEVER_ARM_WARNING   = "Antenna Lever Arm Warning"
string STATUS_FLAGS_GQ7_MOUNTING_TRANSFORM_WARNING  = "Mounting Transform Warning"
string STATUS_FLAGS_GQ7_TIME_SYNC_WARNING           = "Time Sync Warning"
string STATUS_FLAGS_GQ7_SOLUTION_ERROR              = "Solution Error"