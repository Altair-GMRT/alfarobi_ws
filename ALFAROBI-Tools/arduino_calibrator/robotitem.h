#ifndef ROBOTITEM_H
#define ROBOTITEM_H

#include <QGraphicsItem>
#include <QPainter>

class RobotItem : public QGraphicsItem
{
public:

  RobotItem();
  ~RobotItem();

  QRectF boundingRect() const Q_DECL_OVERRIDE;
  void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
  void setAngle(double x) { angle = x;}

private:
  double angle;
  QPixmap robot;

};


#endif // ROBOTITEM_H
