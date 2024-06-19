#include "arduinocontroller.h"

ArduController::ArduController()
  : rx_state(start_U),
    arduino_state(Ardu_Start_U),
    serial_connected(false),
    data_updated(false),
    offset(0),
    heading(0),
    trueHeading(0),
    running(false)
{
  serial_arduino = new QSerialPort;
  QObject::connect(serial_arduino, SIGNAL(readyRead()), this, SLOT(readSerial()));

  std::string param_path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
  YAML::Node node;
  try
  {
    // load yaml
    node = YAML::LoadFile(param_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node doc = node["arduino_controller"];

  offset = doc["offset"].as<double>();
  displayOffset(offset);


}

bool ArduController::connect(std::string port, int baud_rate)
{
  mutex.lock();
  serial_arduino->setPortName(QString::fromStdString(port));
  serial_arduino->setBaudRate((qint32)baud_rate);
  serial_arduino->setDataBits(QSerialPort::Data8);
  serial_arduino->setStopBits(QSerialPort::OneStop);
  serial_arduino->setParity(QSerialPort::NoParity);
  serial_connected = serial_arduino->open(QIODevice::ReadOnly);
  if(!serial_connected)
    std::cout << serial_arduino->errorString().toStdString() << std::endl;

  mutex.unlock();

  std::cout << "Connected to : " << port << std::endl;

  return serial_connected;

}

bool ArduController::disconnect()
{
  mutex.lock();
  serial_arduino->close();
  mutex.unlock();
  serial_connected = !serial_arduino->isOpen();
  std::cout << "disconnected to : " << serial_arduino->portName().toStdString() << " --> " << serial_connected << std::endl;
  return serial_connected;
}

void ArduController::readSerial()
{
  QByteArray serial_data = serial_arduino->readAll();
  for(auto d : serial_data)
    updateState(d);
}

void ArduController::updateState(uchar data)
{
  switch (arduino_state)
  {
  case Ardu_Start_U :
  {
    temp_data.append(data);
    if(data == 'G')
    {
      roll = temp_data.remove(QRegExp("[UGMDIYAB]"));
      arduino_state = Ardu_Start_G;
      temp_data.clear();
    }
    break;
  }

  case Ardu_Start_G :
  {
    temp_data.append(data);
    if(data == 'M')
    {
      pitch = temp_data.remove(QRegExp("[UGMDIYAB]"));
      arduino_state = Ardu_Start_M;
      temp_data.clear();
    }
    break;

  }

  case Ardu_Start_M :
  {
    temp_data.append(data);
    if(data == 'D')
    {
      yaw = temp_data.remove(QRegExp("[UGMDIYAB]"));
      curr_yaw = yaw.toDouble();
      arduino_state = Ardu_Start_D;
      temp_data.clear();
      double yaw_ = yaw.toDouble() - offset;
      if(yaw_ >= 360) yaw_-=360;
      else if (yaw_ < 0) yaw_ += 360;
      sendRPY(roll.toDouble(), pitch.toDouble(), yaw_);
      roll.clear();
      pitch.clear();
      yaw.clear();
    }
    break;
  }

  case Ardu_Start_D :
  {
    temp_data.append(data);
    if(data == 'I')
    {
      gyroRoll = temp_data.remove(QRegExp("[UGMDIYAB]"));
      arduino_state = Ardu_Start_I;
      temp_data.clear();
    }
    break;
  }
  case Ardu_Start_I :
  {
    temp_data.append(data);
    if(data == 'Y')
    {
      gyroPitch = temp_data.remove(QRegExp("[UGMDIYAB]"));
      arduino_state = Ardu_Start_Y;
      temp_data.clear();
    }
    break;
  }
  case Ardu_Start_Y :
  {
    temp_data.append(data);
    if(data == 'B')
    {
      gyroYaw = temp_data.remove(QRegExp("[UGMDIYAB]"));
      sendGyro(gyroRoll.toDouble(), gyroPitch.toDouble(), gyroYaw.toDouble());
      arduino_state = Ardu_Start_B;
      temp_data.clear();
      gyroPitch.clear();
      gyroRoll.clear();
      gyroYaw.clear();
    }
    break;
  }

  case Ardu_Start_B :
  {
    temp_data.append(data);
    if(data == 'A')
    {
      QString button_ = temp_data.remove(QRegExp("[UGMDIYAB]"));
      button = button_.toInt();
      arduino_state = Ardu_Start_U;
      sendButton(button);
      temp_data.clear();
    }
    break;
  }
  }



}

void ArduController::setOffset()
{

  // perbaiki offset
  offset = curr_yaw;
  displayOffset(offset);

  YAML::Node node;
  std::string param_path;
  param_path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";

  try
  {
    // load yaml
    node = YAML::LoadFile(param_path.c_str());
  } catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load yaml file.");
    return;
  }

  YAML::Node doc = node["arduino_controller"];

  doc["offset"] = offset;

  // output to file
  std::ofstream fout(param_path.c_str());
  fout << node;
}

