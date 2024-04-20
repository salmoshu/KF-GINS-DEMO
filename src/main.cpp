#include <senssync/rs2sync.h>
#include <imulib/imulib.h>
#include <mutex>
#include <thread>
#include <iomanip>

#include <iostream>
#include <fstream>
#include <string>

extern "C" {
    #include <rtklib/rtklib.h>
}

#ifdef WIN32
#define thread_t    HANDLE
#define lock_t      CRITICAL_SECTION
#define initlock(f) InitializeCriticalSection(f)
#define lock(f)     EnterCriticalSection(f)
#define unlock(f)   LeaveCriticalSection(f)
#define FILEPATHSEP '\\'
#else
#define thread_t    pthread_t
#define lock_t      pthread_mutex_t
#endif

static int duration = 120;   // 120s

struct inc_data
{
    double timestamp;
    int week;
    double tow;
    float  da_x;
    float  da_y;
    float  da_z;
    float  dv_x;
    float  dv_y;
    float  dv_z;
};

gtime_t stamp2time(double sec) {
    gtime_t t = {0};
    t.time = (time_t)sec;
    t.sec = sec - (int)sec;

    return t;
}

#ifdef WIN32
DWORD WINAPI saveIMU(void *arg)
#else
void *saveIMU(void *arg)
#endif
{
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 200);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 200);

    // Output the frame counter every second, then quit after 10 seconds.
    std::mutex mutex;
    std::string motion_type;

    sync_imu_data last_imu_msg = {0};
    inc_data res_data;
    gtime_t imu_time;

    std::string respath =  "../dataset/IMU_LOG.txt";

	std::ofstream outFile(respath, std::ios::out);

    auto callback = [&](rs2::frame frame)
    {
        auto stream = frame.get_profile().stream_type();
        auto stream_index = (stream == GYRO.first)?GYRO:ACCEL;
        // double frame_time = frame.get_timestamp();
        double frame_time = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_FRAME_TIMESTAMP);
        // double frame_time = frame.get_frame_metadata(rs2_frame_metadata_value::RS2_FRAME_METADATA_SENSOR_TIMESTAMP);

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
                imu_time = stamp2time(res_data.timestamp/1000);
                res_data.tow = time2gpst(imu_time, &res_data.week);
                res_data.da_x = rate2inc(last_imu_msg.accel_data.x, imu_msg.accel_data.x, dt);
                res_data.da_y = rate2inc(last_imu_msg.accel_data.y, imu_msg.accel_data.y, dt);
                res_data.da_z = rate2inc(last_imu_msg.accel_data.z, imu_msg.accel_data.z, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.x, imu_msg.gyro_data.x, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.y, imu_msg.gyro_data.y, dt);
                res_data.dv_x = rate2inc(last_imu_msg.gyro_data.z, imu_msg.gyro_data.z, dt);

                // std::cout << std::setprecision(13) << res_data.timestamp << " " << dt << " "
                //           << res_data.da_x << " "
                //           << res_data.dv_x << " " << std::endl;
            }

            if(last_imu_msg.accel_data.x && last_imu_msg.gyro_data.x) {
                outFile << std::setprecision(13) << res_data.tow << " ";
                outFile << std::setprecision(13) << res_data.da_x << " ";
                outFile << std::setprecision(13) << res_data.da_y << " ";
                outFile << std::setprecision(13) << res_data.da_z << " ";
                outFile << std::setprecision(13) << res_data.dv_x << " ";
                outFile << std::setprecision(13) << res_data.dv_y << " ";
                outFile << std::setprecision(13) << res_data.dv_z << " ";
                outFile << "\n";
        }

            last_imu_msg = imu_msg;

            imu_msgs.pop_front();
        }
    };

    pipe.start(cfg, callback);


    int err = 0;
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(duration));
        pipe.stop();
        break;
    }

    return 0;
}

#ifdef WIN32
DWORD WINAPI saveNEMA(void *arg)
#else
void *saveNEMA(void *arg)
#endif
{
    int strtype[] = {STR_SERIAL, STR_FILE};
    char *strpath[] = {
        "ttyUSB0:115200:8:n:1:off",
        "../dataset/GNSS_LOG.txt"
    };
    int n = 0, buffsize = 4096;
    int cycle = 200;
    uint8_t *buff = (uint8_t *)malloc(buffsize);

    int cputime = 0;
    uint32_t tick = tickget();

    stream_t stream[2];
    for(int i = 0; i < 2; i++)  strinit(&stream[i]);   // 初始化stream_t结构体
    strinitcom();

    /* open streams */
    int rw;
    for(int i = 0; i < 2; i++) {
        rw=i<1?STR_MODE_R:STR_MODE_W;
        if (strtype[i]!=STR_FILE) rw|=STR_MODE_W;
        if (!stropen(&stream[i], strtype[i], rw, strpath[i])) {
            printf("str%d open error path=%s", i, strpath[i]);
            strclose(&stream[i]);
            return 0;
        }
    }

    while (1)
    {
        tick = tickget();
        /* read receiver data from input stream */
        if ((n=strread(&stream[0], buff, buffsize))<=0) {
            continue;
        }

        /* write receiver data to log stream */
        strwrite(&stream[1], buff ,n);

        if ((cputime=(int)(tickget()-tick))>0) sleepms(cputime);
    }
    return 0;
}


// /*
int main() {
    thread_t gnss_thread;
    thread_t imu_thread;
    // create logger threads
#ifdef WIN32
    if (!(gnss_thread=CreateThread(NULL,0,saveNEMA,NULL,0,NULL))) {
#else
    if (pthread_create(&gnss_thread,NULL,saveNEMA, NULL)) {
#endif
        // for (i=0;i<MAXSTRRTK;i++) strclose(svr->stream+i);
        printf("GNSS thread create error\n");
        return 0;
    }
// #ifdef WIN32
//     if (!(imu_thread=CreateThread(NULL,0,saveNEMA,NULL,0,NULL))) {
// #else
//     if (pthread_create(&imu_thread,NULL,saveIMU, NULL)) {
// #endif
//         // for (i=0;i<MAXSTRRTK;i++) strclose(svr->stream+i);
//         printf("IMU thread create error\n");
//         return 0;
//     }
    
    saveIMU(NULL);

    return 0;
}
// */