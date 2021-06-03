#include <ros/ros.h>
#include <QGuiApplication>
#include <gui_qt/mainapplication.hpp>

int main(int argc, char **argv) {
  // Set up ROS.
  ros::init(argc, argv, "gui_main");

  QGuiApplication app(argc, argv);
  Main_Application engine;
  engine.run();

  return app.exec();
}
