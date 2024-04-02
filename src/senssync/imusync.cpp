#include "rs2sync.h"

static sync_imu_data CreateUnitedMessage(const CimuData accel_data, const CimuData gyro_data)
{
    sync_imu_data imu_msg;
    // rclcpp::Time t(gyro_data.m_time_ns);  //rclcpp::Time(uint64_t nanoseconds)
    imu_msg.timestamp = gyro_data.m_time_ms;

    imu_msg.gyro_data.x = gyro_data.m_data.x();
    imu_msg.gyro_data.y = gyro_data.m_data.y();
    imu_msg.gyro_data.z = gyro_data.m_data.z();

    imu_msg.accel_data.x = accel_data.m_data.x();
    imu_msg.accel_data.y = accel_data.m_data.y();
    imu_msg.accel_data.z = accel_data.m_data.z();
    return imu_msg;
}

void FillImuData_LinearInterpolation(const CimuData imu_data, std::deque<sync_imu_data>& imu_msgs)
{
    static std::deque<CimuData> _imu_history;
    _imu_history.push_back(imu_data);
    stream_index_pair type(imu_data.m_type);
    imu_msgs.clear();

    if ((type != ACCEL) || _imu_history.size() < 3)
        return;
    
    std::deque<CimuData> gyros_data;
    CimuData accel0, accel1, crnt_imu;

    while (_imu_history.size()) 
    {
        crnt_imu = _imu_history.front();
        _imu_history.pop_front();
        if (!accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel0 = crnt_imu;
        } 
        else if (accel0.is_set() && crnt_imu.m_type == ACCEL) 
        {
            accel1 = crnt_imu;
            const double dt = accel1.m_time_ms - accel0.m_time_ms;

            while (gyros_data.size())
            {
                CimuData crnt_gyro = gyros_data.front();
                gyros_data.pop_front();
                const double alpha = (crnt_gyro.m_time_ms - accel0.m_time_ms) / dt;
                CimuData crnt_accel(ACCEL, lerp(accel0.m_data, accel1.m_data, alpha), crnt_gyro.m_time_ms);
                imu_msgs.push_back(CreateUnitedMessage(crnt_accel, crnt_gyro));
            }
            accel0 = accel1;
        } 
        else if (accel0.is_set() && crnt_imu.m_time_ms >= accel0.m_time_ms && crnt_imu.m_type == GYRO)
        {
            gyros_data.push_back(crnt_imu);
        }
    }
    _imu_history.push_back(crnt_imu);
    return;
}