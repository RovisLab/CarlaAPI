/*
 * ROVIS_TYPES.h
 *
 *  Created on: 28.03.2010
 *      Author: Sorin Grigorescu
 */

#ifndef ROVIS_TYPES_H_
#define ROVIS_TYPES_H_

#include <cstdint>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <ctime>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

// If compiler >= C++ 17, then use the std::variant implementation, otherwise use the mpark implementation stored in rovis include
#include <variant>

// Environment model types
#include <octomap/octomap.h>
#include <octomap/RovisOcTree.h>

// Coordinates transformation class
#include "env/CPose.h"

// OpenCV include
#include <opencv2/core.hpp>

// Logging library spdlog
#include <spdlog/spdlog-inl.h>

#define CACHE_SIZE	10 // Size of the data cache for each filter

#define EULER		2.71828183f
#define PI			3.14159265358979f   // 180 (PI)
#define PI_2        1.57079632679489f   //  90 (PI / 2)
#define TWO_PI		6.28318530717958f   // 360 (2 * PI)
#define DEG2RAD	    0.01745329251994f   // degree to radian (PI / 180)
#define RAD2DEG     57.2957795130824f   // radian to degree (180 / PI)
#define MPS2KMH     3.6f                // meters per second to Kmh and vice-versa
#define KMH2MPS     0.277777f           // Kmh to meters per second (1 / 3.6)
#define SEC2MSEC    1000.f              // seconds to milliseconds
#define MSEC2SEC    0.001f              // milliseconds to seconds (1 / 1000)
#define GRAVITY     9.81f               // earth's gravitational acceleration is approximately 9.81 m/s^2

constexpr auto ROVIS_TRUE = true;
constexpr auto ROVIS_FALSE = false;

typedef uint32_t                        ROVIS_UINT;
typedef int32_t                         ROVIS_INT;
typedef uint64_t                        ROVIS_ULONG;
typedef int64_t                         ROVIS_LONG;
typedef float                           ROVIS_FLOAT;
typedef double                          ROVIS_DOUBLE;
typedef bool                            ROVIS_BOOL;
typedef char                            ROVIS_CHAR;
typedef unsigned char                   ROVIS_UCHAR;
typedef size_t                          ROVIS_SIZE_T;
typedef std::chrono::milliseconds::rep  ROVIS_TIME_UNIT;
typedef std::atomic<ROVIS_BOOL>         ROVIS_ATOMIC_BOOL;
typedef std::atomic<ROVIS_INT>          ROVIS_ATOMIC_INT;
typedef std::atomic<ROVIS_DOUBLE>       ROVIS_ATOMIC_DOUBLE;
typedef std::atomic<ROVIS_TIME_UNIT>    ROVIS_ATOMIC_TIME_UNIT;
typedef std::string                     ROVIS_STRING;
typedef std::thread                     ROVIS_THREAD;
typedef std::mutex                      ROVIS_MUTEX;

enum ROVIS_DATA_TYPE
{
    // Base types
    ROVIS_UNDEFINED			    = -1,
    ROVIS_RAW_DATA_STREAM	    = 0,
    ROVIS_VECTOR_BOOL           = 29,
    ROVIS_VECTOR_INT            = 1,
    ROVIS_VECTOR_FLOAT          = 2,
    ROVIS_VECTOR_DOUBLE         = 3,
    ROVIS_VECTOR_STRING         = 25,

    // Sensor data types
    ROVIS_IMAGE                 = 4,
    ROVIS_RADAR                 = 5,
    ROVIS_ULTRASONICS           = 6,
    ROVIS_IMU                   = 7,
    ROVIS_GPS                   = 8,

    // Perception types
    ROVIS_POINTS                = 9,
    ROVIS_VOXELS                = 10,
    ROVIS_POSES_6D              = 11,
    ROVIS_2D_ROIS               = 12,
    ROVIS_3D_BBOXES             = 13,
    ROVIS_POINT_FEATURES        = 14,
    ROVIS_OCTREE                = 15,
    ROVIS_GRIDMAP               = 16,
    ROVIS_LANES_MODEL           = 17,
    ROVIS_SLAM                  = 23,

    // Deep learning types
    ROVIS_DNN_OUTPUT            = 18,

    // Control types
    ROVIS_REFERENCE_SETPOINTS   = 19,
    ROVIS_STATE                 = 20,
    ROVIS_CONTROL_INPUT         = 21,
    ROVIS_MEASUREMENT           = 22,
    ROVIS_ESTIMATED_TRAJECTORY  = 24,
    ROVIS_LANDMARKS             = 26,

    // Communication types
    ROVIS_TERMINAL_DATA         = 28,

    // Wheel stuck
    ROVIS_WHEEL_STUCK_MESSAGE   = 27,
};

enum ROVIS_FILTER_TYPE
{
    // Base undefined type filter
    ROVIS_UNDEFINED_FILTER_TYPE				        = -1,
    
    // Sensor filters
    ROVIS_IMAGE_READER_FILTER_TYPE			        = 0,
    ROVIS_MONO_CAMERA_FILTER_TYPE                   = 1,
    ROVIS_RGBDCAMERA_FILTER_TYPE                    = 2,
    ROVIS_BUMBLEBEE_CAMERA_FILTER_TYPE		        = 3,
    ROVIS_GPS_FILTER_TYPE                           = 4,
    ROVIS_IMU_FILTER_TYPE                           = 5,
    ROVIS_COMPASS_FILTER_TYPE                       = 60,
    ROVIS_RADAR_FILTER_TYPE                         = 6,
    ROVIS_LIDAR_FILTER_TYPE                         = 7,
    ROVIS_ULTRASONICS_FILTER_TYPE                   = 8,
    ROVIS_TARAXL_FILTER_TYPE                        = 9,
    ROVIS_GRASSHOPPER_FILTER_TYPE                   = 10,
    ROVIS_QUADCAM_FILTER_TYPE                       = 11,
    ROVIS_ROTARY_ENCODER_FILTER_TYPE                = 36,

    // Deep learning filters
    ROVIS_DNN_ROAD_DRIVING_FILTER_TYPE              = 12,
    ROVIS_DNN_DETECTRON2_FILTER_TYPE                = 43,
    ROVIS_DNN_ONNX_FILTER_TYPE                      = 48,

    // Localization and mapping filters
    ROVIS_ARUCO_LOCALIZATION_FILTER_TYPE            = 14,
    ROVIS_VISUAL_SLAM_FILTER_TYPE                   = 15,
    ROVIS_SVO_TRACKER_DATA_FILTER_TYPE              = 16,
	
    // Perception filters
    ROVIS_BINARY_SEGMENTATION_FILTER_TYPE	        = 17,
    ROVIS_ARUCO_DETECTION_FILTER_TYPE               = 20,
    ROVIS_POINTS_TRACKER_FILTER_TYPE                = 35,
    ROVIS_OBJECT_DETECTOR_2D_FILTER_TYPE		    = 18,
    ROVIS_OBJECT_DETECTOR_3D_FILTER_TYPE            = 37,
    ROVIS_OBJECT_TRACKER_FILTER_TYPE                = 19,
    ROVIS_LANE_DETECTION_FILTER_TYPE                = 23,
    ROVIS_SEMANTIC_SEGMENTATION_FILTER_TYPE         = 24,
    ROVIS_QRCODE_READER_FILTER_TYPE                 = 55,
    ROVIS_SVA_FILTER_TYPE                           = 59,
	
    // Environment modelling filter types
    ROVIS_SENSOR_FUSION_MODEL_FILTER_TYPE           = 22,
    ROVIS_MAP_FILTER_TYPE                           = 47,
	
    // Mission planning filters
    ROVIS_VEHICLE_MISSION_PLANNER_FILTER_TYPE       = 25,
    ROVIS_VEHICLE_LOCAL_PLANNER_FILTER_TYPE         = 26,
    ROVIS_VEHICLE_UNSTUCK_FILTER_TYPE               = 38,
    ROVIS_DRONE_WAYPOINTS_PLANNER_FILTER_TYPE       = 41,
	
    // Control filters
    ROVIS_VEHICLE_SIMULATION_FILTER_TYPE            = 29,
    ROVIS_VEHICLE_CONTROL_FILTER_TYPE               = 27,
    ROVIS_VEHICLE_CONTROL_DIFF_FILTER_TYPE          = 44,
    ROVIS_VEHICLE_STATE_ESTIMATION_FILTER_TYPE      = 13,
    ROVIS_DRONE_SIMULATION_FILTER_TYPE              = 39,
    ROVIS_DRONE_CONTROL_FILTER_TYPE                 = 28,
    ROVIS_DRONE_STATE_ESTIMATION_FILTER_TYPE        = 40,
    ROVIS_ARM_SIMULATION_FILTER_TYPE                = 50,
    ROVIS_ARM_CONTROL_FILTER_TYPE                   = 51,
    ROVIS_ARM_STATE_ESTIMATION_FILTER_TYPE          = 52,
    ROVIS_INERTIAL_ODOMETRY_FILTER_TYPE             = 53,
    ROVIS_LEGGED_SIMULATION_FILTER_TYPE             = 65,
    ROVIS_LEGGED_CONTROLLER_FILTER_TYPE             = 66,
    ROVIS_LEGGED_CONTROLLER_LOWLEVEL_FILTER_TYPE    = 69,
    ROVIS_LEGGED_STATE_ESTIMATION_FILTER_TYPE       = 67,

    // Actuator filters
    ROVIS_VEHICLE_ACTUATOR_FILTER_TYPE              = 30,
    ROVIS_DRONE_ACTUATOR_FILTER_TYPE                = 42,
    ROVIS_ARM_ACTUATOR_FILTER_TYPE                  = 49,
    ROVIS_LEGGED_ACTUATOR_FILTER_TYPE               = 64,

    // Vizualization filters
    ROVIS_VIZ_SENSING_FILTER_TYPE                   = 45,
    ROVIS_VIZ_VEHICLE_FILTER_TYPE                   = 46,
    ROVIS_VIZ_OCTOVIZ_FILTER_TYPE                   = 54,
    ROVIS_VIZ_INVENTORY_FILTER_TYPE                 = 56,
    ROVIS_VIZ_MIDS_FILTER_TYPE                      = 63,

    // Terminals
    ROVIS_TERMINAL_DRONE_FILTER_TYPE                = 57,
    ROVIS_TERMINAL_VEHICLE_FILTER_TYPE              = 58,
    ROVIS_TERMINAL_ROVIS_CORE_DAEMON_FILTER_TYPE    = 62,

    // Daemons
    ROVIS_DAEMON_ROVIS_CORE                         = 61,
	
    // Binding filters
    ROVIS_ROS_IMAGE_FILTER_TYPE                     = 31,
    ROVIS_ROS_SYNC_IMAGES_FILTER_TYPE               = 32,
    
    ROVIS_OBJECT_ESTIMATION_FILTER_TYPE             = 33,
    ROVIS_ULTRASONIC_PREDICTION_FILTER_TYPE         = 34,
};

class CBaseRovisFilter;

/*
 * Blockchain key used for storing entries in the blockchain
 * An entry is defined by the VisionCore's ID and the Filter's ID
 * The blockchain key must be unique
 */
struct RovisBlockchainKey
{
    ROVIS_ULONG nCoreID;
    ROVIS_INT   nFilterID;

    RovisBlockchainKey() :
        nCoreID(0),
        nFilterID(-1)
    {}

    RovisBlockchainKey(ROVIS_INT _core_id, ROVIS_INT _filter_id) :
            nCoreID(_core_id),
            nFilterID(_filter_id)
    {}

    ROVIS_BOOL operator==(const RovisBlockchainKey& key) const
    {
        return (nCoreID == key.nCoreID && nFilterID == key.nFilterID);
    }
};
typedef std::vector<RovisBlockchainKey> RovisBlockchainKeys;

// Hash key for undordered maps based on RovisBlockchainKey
template <>
struct std::hash<RovisBlockchainKey>
{
    std::size_t operator()(const RovisBlockchainKey& k) const
    {
        using std::size_t;
        using std::hash;

        // Compute individual hash values for nCoreID and nFilterID and combine them using XOR and bit shifting
        return ((std::hash<ROVIS_ULONG>()(k.nCoreID) ^ (std::hash<ROVIS_INT>()(k.nFilterID) << 1)) >> 1);
    }
};

/*
 * Blockchain entry
 * Maps a blockhain key to a Filter
 */
struct RovisBlockchainEntry
{
    RovisBlockchainKey	Key;
    CBaseRovisFilter*	pRovisFilter;
};

typedef std::vector<RovisBlockchainEntry> RovisBlockchainEntries;

struct RovisBlockchainEntryInfo
{
    RovisBlockchainEntryInfo(RovisBlockchainKey	key, ROVIS_FILTER_TYPE filterType, ROVIS_DATA_TYPE dataType, ROVIS_BOOL enabled, ROVIS_BOOL running, ROVIS_DOUBLE samplingTime)
    {
        this->Key = key;
        this->filterType = filterType;
        this->dataType = dataType;
        this->enabled = enabled;
        this->running = running;
        this->samplingTime = samplingTime;
    }
    RovisBlockchainKey  Key;
    ROVIS_FILTER_TYPE   filterType;
    ROVIS_DATA_TYPE     dataType;
    ROVIS_BOOL          enabled;
    ROVIS_BOOL          running;
    ROVIS_DOUBLE        samplingTime;
};
typedef std::vector<RovisBlockchainEntryInfo> RovisBlockchainEntriesInfo;

/*
 * Control types
 */
struct RovisReferenceSetPoints
{
    // Reference vector calculated via planning (control reference)
    std::vector<Eigen::VectorXf> ref;

    // Samples of reference state trajectories (used mainly for visualization)
    std::vector<std::vector<Eigen::VectorXf>> ref_samples;

    // unique ID for the reference trajectory
    ROVIS_INT id;
};

struct RovisLandmark
{
    CPose                           pose;
    ROVIS_FLOAT                     travel_time = 0.f;
    std::vector<Eigen::Vector4f>    waypoints;
};
typedef std::unordered_map<ROVIS_INT, RovisLandmark> RovisLandmarks;

struct RovisTerminalCommand
{
    std::vector<ROVIS_INT> cmd;
    RovisLandmarks landmarks;
};

struct RovisState
{
    RovisState()
    {}

    RovisState(const ROVIS_UINT _num_state_variables)
    {
        this->x_hat = Eigen::VectorXf::Zero(_num_state_variables);
    }

    Eigen::VectorXf x_hat; // State vector
};

struct RovisMeasurement
{
    RovisMeasurement()
    {}

    RovisMeasurement(const ROVIS_UINT _num_measurement_variables)
    {
        this->y_hat = Eigen::VectorXf::Zero(_num_measurement_variables);
    }

    Eigen::VectorXf y_hat; // Measurement vector (output)
};

struct RovisControlInput
{
    RovisControlInput()
    {}

    RovisControlInput(const ROVIS_UINT _num_control_inputs)
    {
        this->u = Eigen::VectorXf::Zero(_num_control_inputs);
    }
    // Control input calculated for setpoint 
    Eigen::VectorXf                 u; // u = [thrust, roll_torque, pitch_torque, yaw_torque]
    RovisReferenceSetPoints         ref_pts;
    std::vector<Eigen::VectorXf>    goal_points;
};

/*
 * Sensors data types
 */
 // --- 6D poses vector ---
typedef std::vector<CPose> RovisPoses;

// --- Image ---
struct RovisImage_
{
    ROVIS_UINT	nRows;		    // Number of image rows
    ROVIS_UINT	nCols;		    // Number of image columns
    ROVIS_UINT	nChannels1;     // Number of image channels for the first image
    ROVIS_UINT	nChannels2;     // Number of image channels for the second image
    ROVIS_UINT	nType1;		    // Image type, according to the OpenCV types
	ROVIS_UINT	nType2;		    // Image type, for second image, according to the OpenCV types
    ROVIS_UINT	nTotalSize1;    // Total image size
	ROVIS_UINT	nTotalSize2;    // Total image size for second image
    ROVIS_BOOL	bIsStereo;	    // True for stereo or RGB-D images
    void*		pData1;		    // Pointer to the RGB image data, or left RGB image in case of a stereo image
    void*		pData2;		    // Pointer to the right RGB image in case of a stereo image, or depth image in case of RGB-D images
    RovisBlockchainKey key;     // Key of the image source filter

	RovisImage_()
	    : nRows(0),
        nCols(0),
        nChannels1(0),
        nChannels2(0),
        nType1(0),
        nType2(0),
        nTotalSize1(0),
        nTotalSize2(0),
        bIsStereo(false),
        pData1(nullptr),
        pData2(nullptr)
    {}

    explicit RovisImage_(const cv::Mat& rgb) :
        nChannels2(0),
        nType2(0),
        nTotalSize2(0),
        bIsStereo(false),
        pData2(nullptr)
    {
        nRows = rgb.rows;
        nCols = rgb.cols;
        nChannels1 = rgb.channels();
        nType1 = rgb.type();
        nTotalSize1 = (ROVIS_UINT)(rgb.total() * rgb.elemSize());

        pData1 = malloc(nTotalSize1);
        memcpy(pData1, rgb.data, nTotalSize1);
    }

    RovisImage_(const cv::Mat& rgb, const cv::Mat& depth) :
        RovisImage_(rgb)
    {
        bIsStereo = ROVIS_TRUE;

        nChannels2 = depth.channels();
        nType2 = depth.type();
        nTotalSize2 = (ROVIS_UINT)(depth.total() * depth.elemSize());

        pData2 = malloc(nTotalSize2);
        memcpy(pData2, depth.data, nTotalSize2);
    }

	RovisImage_(const RovisImage_& other):
		bIsStereo(other.bIsStereo),
		nCols(other.nCols),
		nRows(other.nRows),
        nChannels1(other.nChannels1),
        nChannels2(other.nChannels2),
		nTotalSize1(other.nTotalSize1),
		nTotalSize2(other.nTotalSize2),
		nType1(other.nType1),
		nType2(other.nType2),
		pData1(nullptr),
		pData2(nullptr),
        key(other.key)
	{
		if (other.pData1 != nullptr)
		{
			pData1 = malloc(other.nTotalSize1);
			memcpy(pData1, other.pData1, other.nTotalSize1);
		}

		if (other.pData2 != nullptr)
		{
			pData2 = malloc(other.nTotalSize2);
			memcpy(pData2, other.pData2, other.nTotalSize2);
		}
	}

	RovisImage_(RovisImage_&& other) noexcept:
		bIsStereo(other.bIsStereo),
		nCols(other.nCols),
		nRows(other.nRows),
        nChannels1(other.nChannels1),
        nChannels2(other.nChannels2),
		nTotalSize1(other.nTotalSize1),
		nTotalSize2(other.nTotalSize2),
		nType1(other.nType1),
		nType2(other.nType2),
		pData1(other.pData1),
		pData2(other.pData2),
        key(other.key)
	{
        other.pData1 = nullptr;
        other.pData2 = nullptr;
	}

	RovisImage_& operator=(RovisImage_&& other) noexcept
    {
		if (this != &other) 
        {
            bIsStereo = other.bIsStereo;
			nCols = other.nCols;
			nRows = other.nRows;
            nChannels1 = other.nChannels1;
            nChannels2 = other.nChannels2;
			nTotalSize1 = other.nTotalSize1;
			nTotalSize2 = other.nTotalSize2;
			nType1 = other.nType1;
			nType2 = other.nType2;

			pData1 = other.pData1;
			pData2 = other.pData2;

			other.pData1 = nullptr;
			other.pData2 = nullptr;

            key = other.key;
		}

		return *this;
	}

	RovisImage_& operator=(const RovisImage_& other) 
    {
		if (this != &other)
        {
            bIsStereo = other.bIsStereo;
			nCols = other.nCols;
			nRows = other.nRows;
            nChannels1 = other.nChannels1;
            nChannels2 = other.nChannels2;
			nTotalSize1 = other.nTotalSize1;
			nTotalSize2 = other.nTotalSize2;
			nType1 = other.nType1;
			nType2 = other.nType2;
            key = other.key;
			
			if (pData1 != nullptr)
            {
			    free(pData1);
                pData1 = nullptr;
            }

			if (pData2 != nullptr)
            {
			    free(pData2);
                pData2 = nullptr;
            }

			if (other.pData1 != nullptr)
			{
				pData1 = malloc(other.nTotalSize1);
				memcpy(pData1, other.pData1, other.nTotalSize1);
			}

			if (other.pData2 != nullptr)
			{
				pData2 = malloc(other.nTotalSize2);
				memcpy(pData2, other.pData2, other.nTotalSize2);
			}
		}

		return *this;
	}

    ~RovisImage_()
    {
        if (pData1 != nullptr)
        {
			free(pData1);		 
			pData1 = nullptr;
        }

        if (pData2 != nullptr)
        {
			free(pData2);
			pData2 = nullptr;
        }
    }

    ROVIS_BOOL empty1() const
    {
        return pData1 == 0 || nRows * nCols == 0 || nChannels1 == 0;
    }
    ROVIS_BOOL empty2() const
    {
        return pData2 == 0 || nRows * nCols == 0 || nChannels2 == 0;
    }
};
typedef std::vector<RovisImage_> RovisImages;

// --- 2D image point ---
struct RovisPoint
{
    RovisPoint() :
        id(-1), score(-1.f)
    {
        pt2d = Eigen::Vector2f{ -1.f, -1.f };
        key = RovisBlockchainKey{ -1, -1 };
    }

    RovisPoint(Eigen::Vector2f _pt2D, ROVIS_INT _id = -1, ROVIS_FLOAT _score = -1.f, RovisBlockchainKey _key = RovisBlockchainKey{ -1, -1 }) :
        pt2d(_pt2D), id(_id), score(_score), key(_key)
    {}
	
	RovisPoint(ROVIS_FLOAT _x, ROVIS_FLOAT _y, ROVIS_INT _id = -1, ROVIS_FLOAT _score = -1.f, RovisBlockchainKey _key = RovisBlockchainKey{ -1, -1 }) :
        id(_id), score(_score), key(_key)
    {
		pt2d = Eigen::Vector2f {_x, _y };
	}
	
	Eigen::Vector2f pt2d;
	ROVIS_INT		id;
    ROVIS_FLOAT     score;

    RovisBlockchainKey key;  // Key of the image source filter
};
typedef std::vector<RovisPoint> RovisPoints;


// --- Voxels (point cloud) ---
struct RovisVoxel
{
    RovisVoxel() :
        id(-1), score(-1.f)
    {
        pt3d = Eigen::Vector4f{ 0.f, 0.f, 0.f, 0.f };
    }

    RovisVoxel(Eigen::Vector4f _pt3D, ROVIS_INT _id = -1, ROVIS_FLOAT _score = -1.f) :
        pt3d(_pt3D), id(_id), score(_score)
    {}

    RovisVoxel(ROVIS_FLOAT _x, ROVIS_FLOAT _y, ROVIS_FLOAT _z, ROVIS_FLOAT _w = 1.f, ROVIS_INT _id = -1, ROVIS_FLOAT _score = -1.f) :
        id(_id), score(_score)
    {
        pt3d = Eigen::Vector4f{ _x, _y, _z, _w };
    }

    Eigen::Vector4f pt3d;
    ROVIS_INT		id;
    ROVIS_FLOAT     score;
};
typedef std::vector<RovisVoxel> RovisVoxels;

// --- Radar point ---
struct RovisRadarPoint
{
    RovisRadarPoint() :
        velocity(0.0f), azimuth(0.0f), altitude(0.0f), depth(0.0f)
    {}

    RovisRadarPoint(ROVIS_FLOAT _velocity, ROVIS_FLOAT _azimuth, ROVIS_FLOAT _altitude, ROVIS_FLOAT _depth) :
        velocity(_velocity), azimuth(_azimuth), altitude(_altitude), depth(_depth)
    {}

    ROVIS_FLOAT velocity; // Velocity towards the sensor.
    ROVIS_FLOAT azimuth;  // Azimuth angle in radians.
    ROVIS_FLOAT altitude; // Altitude angle in radians.
    ROVIS_FLOAT depth;    // Distance in meters.
};
typedef std::vector<RovisRadarPoint> RovisRadarPoints;

// --- Ultrasonics ---
struct RovisUltrasonic_
{
    ROVIS_FLOAT range;
    CPose       pose;
	ROVIS_FLOAT max_range;
};
typedef std::vector<RovisUltrasonic_> RovisUltrasonics;

typedef std::vector<RovisUltrasonics> RovisUltrasonicPredictions;

// --- GPS ---
struct RovisGps
{
    ROVIS_FLOAT baseLat;
    ROVIS_FLOAT baseLng;
    ROVIS_FLOAT baseAlt;
    ROVIS_FLOAT lat;
    ROVIS_FLOAT lng;
    ROVIS_FLOAT alt;
};

// --- IMU ---
struct RovisImu
{
    RovisImu()
    {
        this->accX = this->accY = this->accZ = 0.f;
        this->gyroX = this->gyroY = this->gyroZ = 0.f;
        this->yaw = this->pitch = this->roll = 0.f;
    }

    ROVIS_FLOAT accX, accY, accZ;
    ROVIS_FLOAT gyroX, gyroY, gyroZ;
    ROVIS_FLOAT yaw, pitch, roll;
};

// TBD: use RovisVoxel instead
struct RovisLidarPoint
{
    ROVIS_FLOAT angle;
    ROVIS_FLOAT distance;
    ROVIS_FLOAT quality;
};
typedef std::vector<RovisLidarPoint> RovisLidarPoints;

/*
 * Perception types
 */
typedef octomap::RovisOcTree RovisOcTree;

struct RovisEnvironment
{
    CPose                           pose;
    std::unique_ptr<RovisOcTree>    pOccupancyModel;

    RovisEnvironment(ROVIS_FLOAT _resolution)
    {
        pOccupancyModel = std::make_unique<RovisOcTree>(_resolution);
    }

    RovisEnvironment(const RovisEnvironment& from)
    {
        from.copyTo(*this);
    }

    RovisEnvironment(RovisEnvironment&&) = default;

    RovisEnvironment& operator=(const RovisEnvironment& from)
    {
        if (this != &from)
        {
            from.copyTo(*this);
        }

        return *this;
    }

    RovisEnvironment& operator=(RovisEnvironment&&) = default;

    void copyTo(RovisEnvironment& dest) const
    {
        dest.pose = pose;
        dest.pOccupancyModel = std::make_unique<RovisOcTree>(pOccupancyModel->getResolution());

        // Information about color is lost when the copy constructor/operator is used
        // Maybe implement an efficient copy constructor for ColorOcTree?
        for (auto it = pOccupancyModel->begin_leafs(); it != pOccupancyModel->end_leafs(); ++it)
        {
            auto* node = dest.pOccupancyModel->updateNode(it.getCoordinate().x(), it.getCoordinate().y(), it.getCoordinate().z(), ROVIS_TRUE, ROVIS_TRUE);
            node->setValue(it->getValue());
            node->setColor(it->getColor());
            node->setObjectClass(it->getObjectClass());
        }
    }
};

struct RovisRoi2D
{
    RovisRoi2D(Eigen::Vector2f _origin, ROVIS_FLOAT _width, ROVIS_FLOAT _height, ROVIS_FLOAT _conf = 0.F, ROVIS_INT _cls = -1, ROVIS_INT _id = -1)
    {
        this->cls = _cls; 
        this->id = _id;
        this->confidence = _conf;
        this->origin = std::move(_origin);
        this->width = _width;
        this->height = _height;
    }

    RovisRoi2D()
    {
        this->cls = -1; 
        this->id = -1;
        this->confidence = 0.F;
        this->origin = Eigen::Vector2f(0, 0);
        this->width = 0;
        this->height = 0;
    }

    // Top left corner is considered as origin
    ROVIS_INT       id;
    ROVIS_INT       cls;
    ROVIS_FLOAT     confidence;
    Eigen::Vector2f origin;
    ROVIS_FLOAT     width;
    ROVIS_FLOAT     height;

    // Key of the image source filter for which the ROI was computed
    RovisBlockchainKey key;
};
typedef std::vector<RovisRoi2D> RovisRois2D;

struct RovisBBox3D
{	
	struct RovisBBoxColor
	{
		ROVIS_FLOAT red = 0.0;
		ROVIS_FLOAT green = 255.0;
		ROVIS_FLOAT blue = 0.0;
	};

	RovisBBox3D(CPose _origin, ROVIS_FLOAT _width, ROVIS_FLOAT _height, ROVIS_FLOAT _depth,  ROVIS_INT _cls = -1, ROVIS_INT _id = -1)
	{
        this->id = _id;
		this->origin = std::move(_origin);
		this->width = _width;
		this->height = _height;
		this->depth = _depth;
		this->cls = _cls;
	}

	RovisBBox3D()
    {
        this->id = -1;
        this->width = 0.f;
        this->height = 0.f;
        this->depth = 0.f;
        this->cls = -1;
    }

	// Top left corner is considered as origin
    ROVIS_INT       id;
	RovisBBoxColor  color;
	CPose    		origin;
	ROVIS_FLOAT		width;
	ROVIS_FLOAT		height;
	ROVIS_FLOAT		depth;
	ROVIS_INT	    cls;

    // Key of the image source filter for which the ROI was computed
    RovisBlockchainKey key;
};
typedef std::vector<RovisBBox3D> RovisBBoxes3D;

// TODO: use RovisReferenceSetpoints instead
using RovisTrajectory = std::vector<Eigen::VectorXf>;
struct RovisObjectsTrajectory
{
    RovisBBoxes3D objects;
    std::vector<RovisTrajectory> trajectories;
};

struct RovisLane
{
    RovisLane()
    {
        this->id = -1;
    }

    RovisLane(ROVIS_INT _id)
    {
        this->id = _id;
    }

    RovisLane(ROVIS_INT _id, Eigen::Vector4f _model)
    {
        this->id = _id;
        this->model = _model;
    }

    ROVIS_INT       id;
    Eigen::Vector4f model;
};
typedef std::vector<RovisLane> RovisLanesModel;

struct RovisSlam
{
    CPose       pose;       // Absolute pose between 2 consecutive frames
    RovisPoints keypoints;  // Number of keypoints and voxels must be equal
    RovisVoxels voxels;
};

/*
 * Deep learning types
 */
struct RovisDnnBranchIO
{
    RovisDnnBranchIO()
    {
        this->id = ROVIS_UNDEFINED;
        this->data_type = ROVIS_UNDEFINED;
    }

    ROVIS_INT       id;
    ROVIS_DATA_TYPE data_type;

    std::vector<
        std::variant<
            std::vector<ROVIS_FLOAT>,
            RovisRois2D,
            RovisBBoxes3D,
            RovisLanesModel,
            RovisImages
        >
    > data;
};
typedef std::vector<RovisDnnBranchIO> RovisDnnIO;

#endif /* ROVIS_TYPES_H_ */
