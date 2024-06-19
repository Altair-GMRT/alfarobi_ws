#ifndef ARDUINOCONTROLLER_H
#define ARDUINOCONTROLLER_H

#include <QSerialPort>
#include <QSerialPortInfo>
#include <QObject>
#include <QIODevice>
#include <iostream>
#include <thread>
#include <mutex>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>

class ArduController : public QObject
{
    Q_OBJECT

public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::duration<double,std::milli> Duration;

    enum rx_state_t {
        start_U,
        start_G,
        start_M,
        checksum
    };

    enum rx_arduino_state {
      Ardu_Start_U,
      Ardu_Start_G,
      Ardu_Start_M,
      Ardu_Start_D,
      Ardu_Start_I,
      Ardu_Start_Y,
      Ardu_Start_B,
    };


    ArduController();
//    ~ArduController();

    bool connect(std::string port, int baud_rate = 115200);
    bool disconnect();
    void setOffsetCompass();
    double getCompass(){ return trueHeading; }
    bool isoffset(){ return offsetSet;}

    void start()
    {
        running = true;
        thread = std::thread(&ArduController::loop, this);
    }

    void stop()
    {
        running = false;
    }

    bool serial_connected;


public slots:
    void readSerial();
    void setOffset();

signals:
    void sendRPY(double roll, double pitch, double yaw);
    void sendGyro(double x, double y, double z);
    void sendButton(int x);
    void displayOffset(double x);

private:

    QSerialPort *serial_arduino;
    bool data_updated;
    bool running;
    bool writing_serial;
    std::thread thread;
    std::mutex mutex;
    rx_state_t rx_state;
    rx_arduino_state arduino_state;
    double offset;
    double heading;
    double trueHeading;
    double curr_yaw;

    Clock::time_point update_data_time_point;
    Duration wait_ms;
    QString roll,pitch,yaw,gyroRoll,gyroPitch,gyroYaw, temp_data;
    int button;

    std::map<int, std::string> button_mapping;

    bool offsetSet;


    void loop();
    void readSerialArduino();
    void updateState(uchar data);
};

#endif // ARDUINOCONTROLLER_H
