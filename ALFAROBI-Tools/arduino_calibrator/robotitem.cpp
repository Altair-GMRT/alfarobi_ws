#include "robotitem.h"

RobotItem::RobotItem()
{
  QString path = QString("fukuro.png");
  robot = QPixmap(path).scaledToHeight(100, Qt::SmoothTransformation);
}

RobotItem::~RobotItem()
{

}

QRectF RobotItem::boundingRect() const
{
  return QRectF(robot.rect());
}

void RobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
  qreal xc = boundingRect().width()/2;
  qreal yc = -boundingRect().height()/2;

  painter->scale(1,-1);

  QPen pen(Qt::green, 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);

  painter->setPen(pen);
  painter->translate(xc, yc);

  qreal degree = -angle;// * 180/ M_PI;
  painter->rotate(degree);
  painter->drawLine(0, 0, 0 , 120);

//  painter->translate(xc,yc);
  painter->drawPixmap(-boundingRect().width()/2, -boundingRect().height()/2, robot);
}



