//#include <op3_walking_module/robotmodel.h>

//using namespace Eigen;

//namespace robotis_op
//{

//RobotModel::RobotModel()
//{
//  hipWidth = 0.074;
//  legThigh = 0.093;
//  footHeight = 0.0335;

//  //set Trasnlation
//  base = Translation3d(Vector3d(0, 0, 0));
//  com = Translation3d(Vector3d(-0.00177, 0,  -0.0365));
//  lfootStep = Translation3d(Vector3d(0, 0, 0));
//  rfootStep = Translation3d(Vector3d(0, 0, 0));

//  lHip = Translation3d(Vector3d(0, 0.5*hipWidth, 0));
//  rHip = Translation3d(Vector3d(0, -0.5*hipWidth, 0));
//  lKnee = Translation3d(Vector3d(0, 0, -legThigh));
//  rKnee = Translation3d(Vector3d(0, 0, -legThigh));
//  lAnkle = Translation3d(Vector3d(0, 0, -legThigh));
//  rAnkle = Translation3d(Vector3d(0, 0, -legThigh));
//  lFoot = Translation3d(Vector3d(0, 0, -footHeight));
//  rFoot = Translation3d(Vector3d(0, 0, -footHeight));

//  //rotasi tidak perlu diatur
//  // 1 0 0
//  // 0 1 0
//  // 0 0 1

//  supportLegSign = RIGHT_LEG;
//  supportExchange = false;
//  supportExchangeLock = false;

//  // Set the translation of the base frame so that the CoM is at (0,0), and the support foot floor point is on the floor (z = 0)
//  Vector3d comPos = base * com.translation();
//  Vector3d footPos = (supportLegSign == RIGHT_LEG ? rFootPosition() : lFootPosition());
//  base.translation() = Vector3d(-comPos.coeff(0,0), -comPos.coeff(1,0), -footPos.coeff(2,0));

//  lfootStep.translation() = lFootPosition();
//  Quaterniond Qtemp = Quaterniond(lFootOrientation());
//  AngleAxisd lfoot(fusedYaw(Qtemp), Vector3d(0,0,1));
//  lfootStep.linear() =  lfoot.toRotationMatrix();
//  rfootStep.translation() = rFootPosition();
//  Quaterniond Wtemp = Quaterniond(rFootOrientation());
//  AngleAxisd rfoot(fusedYaw(Wtemp), Vector3d(0,0,1));
//  rfootStep.linear() = rfoot.toRotationMatrix();

//}

//RobotModel::~RobotModel()
//{

//}


//void RobotModel::reset()
//{
//  base = Translation3d(Vector3d(0, 0, 0));
////  base.linear() = Quaterniond(1,0,0,0);
//  supportExchange = false;
//  supportExchangeLock = false;
//}

//void RobotModel::resetPosition()
//{
//  base = Translation3d(Vector3d(0, 0, 0));
////  footStep = Translation3d(Vector3d(0, 0, 0));
//}

//void RobotModel::process(const margait_contrib::Pose &pose, double fusedX, double fusedY)
//{
//  applyPose(pose);
//  alignModel(fusedX, fusedY);

//  double footDiff = rFootPosition().coeff(1,0) - lFootPosition().coeff(1,0);
//  int lowestFoot = (footDiff > 0 ? LEFT_LEG : RIGHT_LEG);

//  if(supportExchangeLock && (fabs(footDiff) >= footMinDiff)) //tolong ini di set footMinDiff
//    supportExchangeLock = false;

//  supportExchange = false;
//  if(lowestFoot != supportLegSign && !supportExchangeLock)
//  {
//    supportExchangeLock = true;
//    supportExchange = true;
//    setSupportLeg(lowestFoot);
//  }
//}

//void RobotModel::applyPose(const margait_contrib::Pose &pose)
//{
//  // shoulder: y -> x
//  // hip: z -> x -> y
//  // ankle: y -> x

//  lHip.rotate(AngleAxisd(pose.leftLegPose.hip.z, Vector3d(0, 0, 1))
//              * AngleAxisd(pose.leftLegPose.hip.x, Vector3d(1, 0, 0))
//              * AngleAxisd(pose.leftLegPose.hip.y, Vector3d(0, 1, 0)));

//  rHip.rotate(AngleAxisd(pose.rightLegPose.hip.z, Vector3d(0, 0, 1))
//              * AngleAxisd(pose.rightLegPose.hip.x, Vector3d(1, 0, 0))
//              * AngleAxisd(pose.rightLegPose.hip.y, Vector3d(0, 1, 0)));

//  lKnee.rotate(AngleAxisd(pose.leftLegPose.knee.y, Vector3d(0, 1, 0)));

//  rKnee.rotate(AngleAxisd(pose.rightLegPose.knee.y, Vector3d(0, 1, 0)));

//  lAnkle.rotate(AngleAxisd((pose.leftLegPose.ankle.y), Vector3d(0, 1, 0))
//                * AngleAxisd((pose.leftLegPose.ankle.x), Vector3d(1, 0, 0)));
//  rAnkle.rotate(AngleAxisd((pose.rightLegPose.ankle.y), Vector3d(0, 1, 0))
//                * AngleAxisd((pose.rightLegPose.ankle.x), Vector3d(1, 0, 0)));
//}

//void RobotModel::alignModel(double fusedX, double fusedY)
//{
//  Affine3d *supportFootStep;

//  // Apply the given fused roll/pitch to the model (temporarily annuls the fused yaw of the robot)
//  base.linear() =  quatFromFusedPR(fusedX, fusedY).toRotationMatrix();

//  if(supportLegSign == RIGHT_LEG)
//  {
//    tempRfootPoint = rFoot;
//    supportFootStep = &rfootStep;

//    // Calculate the difference in fused yaw (CCW) from the support foot floor point frame to the current support footstep frame, and yaw the base frame by this value
//    double yawDelta  = fusedYaw(Quaterniond(supportFootStep->rotation())) - fusedYaw(Quaterniond(tempRfootPointOrientation()));

//    Quaterniond tempQuat(cos(0.5*yawDelta), 0, 0, sin(0.5*yawDelta));

//    base.linear() = tempQuat.toRotationMatrix() * base.rotation();

//    // Translate the model (base frame) so that the support foot floor point and support footstep frames coincide in terms of position
//    base.translate(supportFootStep->translation() - tempRfootPointPosition());

//  }

//  else
//  {
//    tempLfootPoint = lFoot;
//    supportFootStep = &lfootStep;

//    // Calculate the difference in fused yaw (CCW) from the support foot floor point frame to the current support footstep frame, and yaw the base frame by this value
//    double yawDelta  = fusedYaw(Quaterniond(supportFootStep->rotation())) - fusedYaw(Quaterniond(tempLfootPointOrientation()));
//    Quaterniond tempQuat(cos(0.5*yawDelta), 0, 0, sin(0.5*yawDelta));
//    base.linear() = tempQuat.toRotationMatrix() * base.rotation();

//    // Translate the model (base frame) so that the support foot floor point and support footstep frames coincide in terms of position
//    base.translate(supportFootStep->translation() - tempLfootPointPosition());
//  }

//}

//void RobotModel::setSupportLeg(int leg)
//{
//  supportLegSign = leg;

//  Vector3d footPos = (supportLegSign == RIGHT_LEG ? rFootPosition() : lFootPosition());

//  base.translate(Vector3d(0, 0, -footPos.coeff(2,0)));
//  lfootStep.translate(Vector3d(0, 0, -footPos.coeff(2,0)));
//  rfootStep.translate(Vector3d(0, 0, -footPos.coeff(2,0)));
//}


//Vector3d RobotModel::suppComVector()
//{
//  Affine3d temp;
//  Vector3d compos;

//  if(supportLegSign == RIGHT_LEG)
//  {
//   temp =  base * rHip * rKnee * rAnkle * rFoot ;
//   compos = temp.inverse() * com.translation();
//  }

//  else
//  {
//    temp =  base * lHip * lKnee * lAnkle * lFoot ;
//    compos = temp.inverse() * com.translation();
//  }

//  return compos;

//}

///*
// *
// * MATH FUNCTION
// * untuk hitung posisi dan orientasi
// *
// */

//Vector3d RobotModel::lFootPosition()
//{
//  Vector3d pos = base * lHip * lKnee * lAnkle * lFoot * Vector3d(0,0,0);
//  return pos;
//}

//Matrix3d RobotModel::lFootOrientation()
//{
//  Affine3d temp;
//  temp = base * lHip * lKnee * lAnkle * lFoot;
//  return temp.rotation();
//}

//Matrix3d RobotModel::lFootRotation()
//{

//}

//Vector3d RobotModel::rFootPosition()
//{
//  Vector3d pos = base * rHip * rKnee * rAnkle * rFoot * Vector3d(0,0,0);
//  return pos;
//}

//Matrix3d RobotModel::rFootOrientation()
//{
//  Affine3d temp;
//  temp = base * rHip * rKnee * rAnkle * rFoot;
//  return temp.rotation();
//}

//Matrix3d RobotModel::rFootRotation()
//{

//}

//Vector3d RobotModel::tempRfootPointPosition()
//{
//  Vector3d pos = base * rHip * rKnee * rAnkle * tempRfootPoint * Vector3d(0,0,0);
//  return pos;
//}

//Matrix3d RobotModel::tempRfootPointOrientation()
//{
//  Affine3d temp;
//  temp = base * rHip * rKnee * rAnkle * tempRfootPoint;
//  return temp.rotation();
//}

//Vector3d RobotModel::tempLfootPointPosition()
//{
//  Vector3d pos = base * lHip * lKnee * lAnkle * tempLfootPoint * Vector3d(0,0,0);
//  return pos;
//}

//Matrix3d RobotModel::tempLfootPointOrientation()
//{
//  Affine3d temp;
//  temp = base * lHip * lKnee * lAnkle * tempLfootPoint;
//  return temp.rotation();
//}

//Quaterniond RobotModel::quatFromFusedPR(double fusedX, double fusedY) // Note: It is assumed that the missing fusedZ = 0 and hemi = 1
//{
//  // Precalculate the sin values
//  double sth  = sin(fusedY);
//  double sphi = sin(fusedX);

//  // Calculate the sine sum criterion
//  double crit = sth*sth + sphi*sphi;

//  // Calculate the tilt angle alpha
//  double alpha = (crit >= 1.0 ? M_PI_2 : acos(sqrt(1.0 - crit)));
//  double halpha  = 0.5*alpha;
//  double chalpha = cos(halpha);
//  double shalpha = sin(halpha);

//  // Calculate the tilt axis gamma
//  double gamma = atan2(sth,sphi);
//  double cgamma = cos(gamma);
//  double sgamma = sin(gamma);

//  // Return the required quaternion orientation (a rotation about (cgamma, sgamma, 0) by angle alpha)
//  return Quaterniond(chalpha, cgamma*shalpha, sgamma*shalpha, 0.0); // Order: x, y, z, w!
//}

//double RobotModel::fusedYaw(const Quaterniond& q)
//{
//  // Calculate the fused yaw
//  double fyaw = 2.0*atan2((double)q.z(), (double)q.w()); // Output of atan2(z,w) is [-pi,pi], so this expression is in [-2*pi,2*pi]
//  if(fyaw > M_PI) fyaw -= 2.0*M_PI;    // Fyaw is now in [-2*pi,pi]
//  if(fyaw <= -M_PI) fyaw += 2.0*M_PI;  // Fyaw is now in (-pi,pi]
//  return fyaw;
//}


//}
