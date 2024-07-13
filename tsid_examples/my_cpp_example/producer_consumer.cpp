#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstdlib> 

// Shared data structure for each IMU sensor
struct IMUData {
    std::time_t now_time_t;
    
    double accel_x;
    double accel_y;
    double accel_z;
    double gyro_x;
    double gyro_y;
    double gyro_z;
    // Add more fields as needed
};

// Global variables for shared data and synchronization
IMUData imuData1, imuData2;
std::mutex mtx;
std::condition_variable cv;
bool dataReady1 = false;
bool dataReady2 = false;

// Function to simulate reading data from IMU sensor 1
void readIMU1() {
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    while (true) {
        // Simulate reading data from the IMU sensor
        IMUData newData;
        auto now = std::chrono::system_clock::now();

    // Convert the time point to time_t, which represents the time in seconds since epoch
        newData.now_time_t=std::chrono::system_clock::to_time_t(now);
        newData.accel_x = (rand()%100); // Replace with actual sensor data
        newData.accel_y = (rand()%100); // Replace with actual sensor data
        newData.accel_z = (rand()%100); // Replace with actual sensor data
        newData.gyro_x = (rand()%100);  // Replace with actual sensor data
        newData.gyro_y = (rand()%100);  // Replace with actual sensor data
        newData.gyro_z = (rand()%100);  // Replace with actual sensor data

        // Lock the mutex to update the shared data
        {
            std::lock_guard<std::mutex> lock(mtx);
            imuData1 = newData;
            dataReady1 = true;
        }

        // Notify the consumer thread that new data is available
        cv.notify_one();

        // Simulate a delay between readings
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// Function to simulate reading data from IMU sensor 2
void readIMU2() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    while (true) {
        // Simulate reading data from the IMU sensor
         IMUData newData;
        auto now = std::chrono::system_clock::now();
    // Convert the time point to time_t, which represents the time in seconds since epoch
        newData.now_time_t=std::chrono::system_clock::to_time_t(now);
        newData.accel_x = (rand()%100); // Replace with actual sensor data
        newData.accel_y = (rand()%100); // Replace with actual sensor data
        newData.accel_z =(rand()%100); // Replace with actual sensor data
        newData.gyro_x = (rand()%100);  // Replace with actual sensor data
        newData.gyro_y = (rand()%100);  // Replace with actual sensor data
        newData.gyro_z = (rand()%100);  // Replace with actual sensor data

        // Lock the mutex to update the shared data
        {
            std::lock_guard<std::mutex> lock(mtx);
            imuData2 = newData;
            dataReady2 = true;
        }

        // Notify the consumer thread that new data is available
        cv.notify_one();

        // Simulate a delay between readings
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

// Function to process IMU data
void processIMU() {
    IMUData Data1;
    IMUData Data2;
    while (true) {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [] { return dataReady1 && dataReady2; });
        Data1=imuData1;
        Data2=imuData2; // Mark the data as processed
        dataReady1 = false;
        dataReady2 = false;
        // Unlock the mutex
        lock.unlock();

        // Process the data
        std::cout << "time: "<<Data1.now_time_t<< " Sensor 1 - Accel: (" << Data1.accel_x << ", " << Data1.accel_y << ", " << Data1.accel_z << ") "
                  << "Gyro: (" << Data1.gyro_x << ", " << Data1.gyro_y << ", " << Data1.gyro_z << ")" << std::endl;
        std::cout << "time: "<<Data2.now_time_t<< " Sensor 2 - Accel: (" << Data2.accel_x << ", " << Data2.accel_y << ", " << Data2.accel_z << ") "
                  << "Gyro: (" << Data2.gyro_x << ", " << Data2.gyro_y << ", " << Data2.gyro_z << ")" << std::endl;

        // Simulate processing time
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

int main() {
    // Create threads for reading and processing IMU data
    std::thread readerThread1(readIMU1);
    std::thread readerThread2(readIMU2);
    std::thread processorThread(processIMU);

    // Join threads (this will make main wait for threads to finish, which they won't in this example)
    readerThread1.join();
    readerThread2.join();
    processorThread.join();

    return 0;
}
