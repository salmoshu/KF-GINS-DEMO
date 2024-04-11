#include <senssync/rs2sync.h>
#include <imulib/imulib.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <iomanip>

struct inc_data
{
    double timestamp;
    float da_x;
    float da_y;
    float da_z;
    float dv_x;
    float dv_y;
    float dv_z;
};

int main() {
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    // Output the frame counter every second, then quit after 10 seconds.
    int counters = 0;
    std::mutex mutex;
    std::string motion_type;

    sync_imu_data last_imu_msg = {0};
    inc_data res_data;

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
            if(last_imu_msg.accel_data.x && last_imu_msg.gyro_data.x) {
                double dt = (imu_msg.timestamp - last_imu_msg.timestamp) / 1000;
                res_data.timestamp = imu_msg.timestamp;
                res_data.da_x = rate2inc(last_imu_msg.accel_data.x, imu_msg.accel_data.x, dt);
                res_data.da_y = rate2inc(last_imu_msg.accel_data.y, imu_msg.accel_data.y, dt);
                res_data.da_z = rate2inc(last_imu_msg.accel_data.z, imu_msg.accel_data.z, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.x, imu_msg.gyro_data.x, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.y, imu_msg.gyro_data.y, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.z, imu_msg.gyro_data.z, dt);

                std::cout << std::setprecision(13) << res_data.timestamp << " " << dt << " "
                          << res_data.da_x << " "
                          << res_data.dv_x << " " << std::endl;
            }

            last_imu_msg = imu_msg;

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