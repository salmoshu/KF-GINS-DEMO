#ifndef RS2SYNC_H
#define RS2SYNC_H

#include <librealsense2/rs.hpp>
#include <deque>
#include <Eigen/Core>

typedef std::pair<rs2_stream, int> stream_index_pair;

const stream_index_pair GYRO{RS2_STREAM_GYRO, 0};
const stream_index_pair ACCEL{RS2_STREAM_ACCEL, 0};

// <gyro_data, accel_data>
struct sync_imu_data
{
    double timestamp;
    rs2_vector gyro_data;
    rs2_vector accel_data;
};

class float3
{
    public:
        float x, y, z;

    public:
        float3& operator*=(const float& factor)
        {
            x*=factor;
            y*=factor;
            z*=factor;
            return (*this);
        }
        float3& operator+=(const float3& other)
        {
            x+=other.x;
            y+=other.y;
            z+=other.z;
            return (*this);
        }
};

class CimuData
{
    public:
        CimuData() : m_time_ms(-1) {};
        CimuData(const stream_index_pair type, Eigen::Vector3d data, double time):
            m_type(type),
            m_data(data),
            m_time_ms(time){};
        bool is_set() {return m_time_ms > 0;};
    public:
        stream_index_pair m_type;
        Eigen::Vector3d   m_data;
        double            m_time_ms;
};

template <typename T> T lerp(const T &a, const T &b, const double t) {
    return a * (1.0 - t) + b * t;
}

void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sync_imu_data>& imu_msgs);


#endif /* RS2SYNC_H */