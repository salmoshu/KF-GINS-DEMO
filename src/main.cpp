#include <senssync/rs2sync.h>
#include <mutex>
#include <thread>


int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    // Output the frame counter every second, then quit after 10 seconds.
    int counters = 0;
    std::mutex mutex;
    std::string motion_type;

    auto callback = [&](rs2::frame frame)
    {
        auto stream = frame.get_profile().stream_type();
        auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
        double frame_time = frame.get_timestamp();

        auto crnt_reading = *(reinterpret_cast<const float3*>(frame.get_data()));
        Eigen::Vector3d v(crnt_reading.x, crnt_reading.y, crnt_reading.z);
        CimuData imu_data(stream_index, v, frame.get_timestamp());
        std::deque<sync_imu_data> imu_msgs;

        FillImuData_LinearInterpolation(imu_data, imu_msgs);

        while (imu_msgs.size())
        {
            sync_imu_data imu_msg = imu_msgs.front();
            // process imu_msg here...
            std::cout << "ACCE2: " << imu_msg.accel_data.x << "\n";
            std::cout << "GYRO2: " << imu_msg.gyro_data.x << "\n";

            counters++;

            imu_msgs.pop_front();
        }
    };

    pipe.start(cfg, callback);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if(counters > 50) {
            pipe.stop();
            break;
        }
    }

    return 0;
}

/**
 * @brief 将IMU数据转换为增量型输出
 *        English Notes
 * */
double rate2increment(double v1, double v2, double dt) {
    return (v1+v2)/2*dt;
}