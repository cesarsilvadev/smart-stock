#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sys/mman.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <cmath>
#include <ctime>
#include <mutex>
#include <condition_variable>

std::atomic<bool> takepic(false);
std::mutex picMutex;

#define kangle kalmanUpdate(gyro_variables[2], dt, angulo, qBias, P_00, P_01, P_10, P_11)

const std::string accel_variables_raw[] = {
    "in_accel_x_raw", "in_accel_y_raw", "in_accel_z_raw"
};

const std::string gyro_variables_raw[] = {
    "in_anglvel_x_raw", "in_anglvel_y_raw", "in_anglvel_z_raw"
};

double angulo = 0.0;
int cont = 0;

const double alpha = 0.8;
double qAngle = 0.2;
double qBias = 0.07;
double rMeasure = 1;
double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;

double kalmanUpdate(double gyroRate, double dt, double& kalmanAngle, double& qBias, double& P_00, double& P_01, double& P_10, double& P_11) {
    kalmanAngle += dt * (gyroRate - qBias);
    P_00 += dt * (dt * P_11 - P_01 - P_10 + qAngle);
    P_01 -= dt * P_11;
    P_10 -= dt * P_11;
    P_11 += qBias * dt;

    double y = angulo - kalmanAngle;
    double K_0 = P_00 / (P_00 + rMeasure);
    double K_1 = P_10 / (P_00 + rMeasure);

    kalmanAngle += K_0 * y;
    qBias += K_1 * y;
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;

    return kalmanAngle;
}

void read_and_print_sensor_data(const std::string& file_path) {
    double accel_variables[3] = {0.0, 0.0, 0.0};
    double gyro_variables[3] = {0.0, 0.0, 0.0};
    double distance = 0.0;
    int loop_count = 0;

    std::ofstream outputFile("/home/engenharia/output.txt", std::ios::app);

    if (!outputFile.is_open()) {
        std::cerr << "Error opening the file." << std::endl;
        return;
    }

    while (true) {
        for (int i = 0; i < 3; ++i) {
            try {
                std::ifstream accel_file(file_path + accel_variables_raw[i]);
                if (accel_file) {
                    double accel_data;
                    accel_file >> accel_data;
                    accel_data = (accel_data / 16384) * 10;
                    accel_variables[i] = accel_data;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error reading accelerometer data: " << e.what() << std::endl;
            }
        }

        for (int i = 0; i < 3; ++i) {
            try {
                std::ifstream gyro_file(file_path + gyro_variables_raw[i]);
                if (gyro_file) {
                    double gyro_data;
                    gyro_file >> gyro_data;
                    gyro_data = (gyro_data / 131);
                    gyro_variables[i] = gyro_data;
                }
            } catch (const std::exception& e) {
                std::cerr << "Error reading gyro data: " << e.what() << std::endl;
            }
        }

        if (std::abs(gyro_variables[2]) < 0.05) {
            gyro_variables[2] = 0.0;
        }

        gyro_variables[2] += 0;

        if (angulo > 60.0) {
            loop_count = 0;
            distance = 0.0;
            angulo = 0.0;

            std::lock_guard<std::mutex> lock(picMutex);
            takepic = true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(1));
        angulo += ((((gyro_variables[2] * 0.0001) * 180) / 3.14)) * 2.43;
	
//	std::cout << angulo << std::endl;

        double dt = 0.000005;
        kalmanUpdate(gyro_variables[2], dt, angulo, qBias, P_00, P_01, P_10, P_11);
        if (std::abs(gyro_variables[2]) < 0.03) {
            cont += 1;
            const double epsilon = 5;
            double angtotal = 0;
            angtotal = kangle;
            if (cont > 20) {
                if (std::abs(angtotal) >= epsilon) {
                    auto agora = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                    std::tm* tempo = std::localtime(&agora);
                    char dataHora[20];
                    std::strftime(dataHora, sizeof(dataHora), "%d/%m/%Y %H:%M:%S", tempo);
                    outputFile << "Angulo de abertura: " << kangle << " Hora: " << dataHora << std::endl;
                }
                angulo = 0;
            }
        }
    }
}

void captureImage(int cam, int cameraNumber) {
    struct v4l2_buffer buffer{};
    memset(&buffer, 0, sizeof(buffer));
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = 0;

    if (ioctl(cam, VIDIOC_QUERYBUF, &buffer) == -1) {
        perror("Error querying buffer");
        close(cam);
        return;
    }

    void* bufferStart = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, cam, buffer.m.offset);
    if (bufferStart == MAP_FAILED) {
        perror("Error mapping buffer");
        close(cam);
        return;
    }

    while (true) {
        if (takepic) {
            auto now = std::chrono::system_clock::now();
            std::time_t timestamp = std::chrono::system_clock::to_time_t(now);
            std::tm tm_timestamp = *std::localtime(&timestamp);
            char timestampStr[20];
            std::strftime(timestampStr, sizeof(timestampStr), "%Y%m%d%H%M%S", &tm_timestamp);
            std::string outputFileName = "captured_image_" + std::string(timestampStr) + "_CAM" + std::to_string(cameraNumber) + ".jpg";

            if (ioctl(cam, VIDIOC_QBUF, &buffer) == -1) {
                perror("Error queuing buffer");
                close(cam);
                break;
            }
            auto startTime = std::chrono::high_resolution_clock::now();
            if (ioctl(cam, VIDIOC_DQBUF, &buffer) == -1) {
                perror("Error capturing frame");
                close(cam);
                break;
            }

            std::ofstream outputFile("/home/engenharia/pictures_accel_gyro/" + outputFileName, std::ios::binary);
            if (!outputFile.is_open()) {
                perror("Error opening output file");
                close(cam);
                return;
            }

            outputFile.write(reinterpret_cast<char*>(bufferStart), buffer.bytesused);
            outputFile.close();

            std::cout << "Captured image saved as " << outputFileName << std::endl;

            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

            std::cout << "Execution time: " << duration.count() << " milliseconds" << std::endl;

            takepic = false;
        }
    }

    munmap(bufferStart, buffer.length);
    close(cam);
}

int configCamera(const char* devicePath) {
    int cam = open(devicePath, O_RDWR);
    if (cam == -1) {
        perror("Error opening V4L2 device");
        return 1;
    }

    struct v4l2_control ctrl;
    ctrl.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl.value = V4L2_EXPOSURE_MANUAL;
    if (ioctl(cam, VIDIOC_S_CTRL, &ctrl) == -1) {
        perror("Error setting exposure control");
        close(cam);
        return 1;
    }

    ctrl.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl.value = 60; // Replace with your desired value
    if (ioctl(cam, VIDIOC_S_CTRL, &ctrl) == -1) {
        perror("Error setting shutter speed control");
        close(cam);
        return 1;
    }

    struct v4l2_format format{};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = 1280;
    format.fmt.pix.height = 720;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;

    if (ioctl(cam, VIDIOC_S_FMT, &format) == -1) {
        perror("Error setting format");
        close(cam);
        return 1;
    }

    struct v4l2_requestbuffers req{};
    req.count = 1;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(cam, VIDIOC_REQBUFS, &req) == -1) {
        perror("Error requesting buffer");
        close(cam);
        return 1;
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(cam, VIDIOC_STREAMON, &type) == -1) {
        perror("Error starting streaming");
        close(cam);
        return 1;
    }

    return cam;
}

int main() {
    takepic = false;

    int cam1 = configCamera("/dev/video0");
    int cam2 = configCamera("/dev/video2");

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    std::thread image_capture_thread1(captureImage, cam1, 1);
    std::thread image_capture_thread2(captureImage, cam2, 2);

    std::string file_path = "/sys/bus/iio/devices/iio:device0/";
    std::thread gyro_accel_sensor_thread(read_and_print_sensor_data, file_path);

    gyro_accel_sensor_thread.join();
    image_capture_thread1.join();
    image_capture_thread2.join();

    if (ioctl(cam1, VIDIOC_STREAMOFF, &type) == -1) {
        perror("Error stopping streaming for cam1");
    }

    if (ioctl(cam2, VIDIOC_STREAMOFF, &type) == -1) {
        perror("Error stopping streaming for cam2");
    }

    close(cam1);
    close(cam2);

    return 0;
}
