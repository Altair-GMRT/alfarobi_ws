#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <iostream>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  use_ros_(false),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  this->setWindowTitle(QString("Arduino Calibrator"));
  ui->tabWidget->setCurrentIndex(0);
  ui->baud_rate_box->setCurrentIndex(2);
  ui->baud_rate_box->addItem(tr("Custom"));

  intValidator = new QIntValidator(0, 4000000, this);

  ui->baud_rate_box->setInsertPolicy(QComboBox::NoInsert);

  scene = new QGraphicsScene(this);
  ui->graphicsView->setScene(scene);

  robot = new RobotItem;
  controller = new ArduController;
  scene->addItem(robot);

  on_refresh_button_clicked();


  chart = new Chart;
  chart_gyro = new Chart;
  chart->setTitle("Roll and Pitch");
  chart_gyro->setTitle("Gyro Data");
  chart->legend()->setVisible(true);
  chart_gyro->legend()->setVisible(true);


  //    chart->legend()->
  chart->legend()->setAlignment(Qt::AlignBottom);
  chart->setAnimationOptions(QChart::AllAnimations);
  chart_gyro->legend()->setAlignment(Qt::AlignBottom);
  chart_gyro->setAnimationOptions(QChart::AllAnimations);

  ui->plotter_imu->setChart(chart);
  ui->plotter_gyro->setChart(chart_gyro);
  ui->plotter_imu->setRenderHint(QPainter::Antialiasing);
  ui->plotter_gyro->setRenderHint(QPainter::Antialiasing);


  connect(controller, SIGNAL(sendRPY(double,double,double)), this, SLOT(updateRPY(double,double,double)));
  connect(controller, SIGNAL(sendGyro(double,double,double)), this, SLOT(updateGyro(double,double,double)));
  connect(controller, &ArduController::sendButton, this, &MainWindow::updateButton);
  connect(ui->baud_rate_box,  static_cast<void (QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
          this, &MainWindow::checkCustomBaudRatePolicy);
  connect(ui->offset_btn, &QPushButton::clicked , controller, &ArduController::setOffset);
  connect(controller, &ArduController::displayOffset, this, &MainWindow::displayOffset);

  isCompas = false;

  std::string default_path = ros::package::getPath("op3_manager") + "/config/GlobalConfig.yaml";
  loadButton(default_path);
}

MainWindow::~MainWindow()
{
  delete ui;
  this->close();
}

void MainWindow::loadButton(const std::string path)
{
  YAML::Node doc;
    try
    {
      // load yaml
      doc = YAML::LoadFile(path.c_str());
    } catch (const std::exception& e)
    {
      ROS_ERROR("Fail to load offset config yaml.");
      return;
    }

    // parse right_arm
    YAML::Node button_mode = doc["button_setting"];
    for (YAML::iterator yaml_it = button_mode.begin(); yaml_it != button_mode.end(); ++yaml_it)
    {
      int index;
      std::string mode;

      index = yaml_it->second.as<int>();
      mode = yaml_it->first.as<std::string>();

      button_mapping.insert(std::make_pair(index, mode));
    }

}

void MainWindow::updateRPY(double roll, double pitch, double yaw)
{
    chart->updateRPY(roll, pitch, yaw);
  updateRobot(yaw);
  ui->plotter_roll_val->setText(QString::number(roll));
  ui->plotter_pitch_val->setText(QString::number(pitch));
  ui->plotter_yaw_val->setText(QString::number(yaw));

}

void MainWindow::updateGyro(double roll, double pitch, double yaw)
{
    chart_gyro->updateGyro(roll, pitch, yaw);

}

void MainWindow::updateRobot(double angle)
{
  robot->setAngle(angle);
  scene->update();
}

void MainWindow::updateButton(int x)
{
std::cout << x << std::endl;
  std::string text;
  std::map<int, std::string>::iterator iter = button_mapping.find(x);
  if (iter != button_mapping.end())
  {
      text = iter->second;
  }
  else text = std::string("None");

  ui->buttonStatus->setText(QString::fromStdString(text));
}


void MainWindow::on_refresh_button_clicked()
{
  ui->device_list->clear();
  serial_port_list.clear();
  for(auto s : QSerialPortInfo::availablePorts())
  {
    ui->device_list->addItem(s.portName()+QString(" : ")+s.manufacturer());
    serial_port_list.push_back(s.portName());
  }
}

void MainWindow::on_connect_button_clicked()
{
  if(serial_port_list.size() > 0)
  {
    QString arduino_port = serial_port_list.at(ui->device_list->currentIndex());
    if(ui->connect_button->text() == QString("Connect"))
    {
      if(controller->connect(arduino_port.toStdString(), ui->baud_rate_box->currentText().toInt()))
        ui->connect_button->setText(QString("Disconnect"));
    }

    else
    {
      if(controller->disconnect())
        ui->connect_button->setText(QString("Connect"));
    }
  }

}

void MainWindow::displayOffset(double x)
{
    ui->offset_value->setText(QString::number(x));
}

void MainWindow::checkCustomBaudRatePolicy(int idx)
{
  bool isCustomBaudRate = !ui->baud_rate_box->itemData(idx).isValid();
  ui->baud_rate_box->setEditable(isCustomBaudRate);
  if (isCustomBaudRate) {
    ui->baud_rate_box->clearEditText();
    QLineEdit *edit = ui->baud_rate_box->lineEdit();
    edit->setValidator(intValidator);
  }
}
