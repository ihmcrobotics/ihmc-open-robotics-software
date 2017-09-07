package us.ihmc.robotics.kinematics.fourbar;

public interface FourbarCalculatorWithDerivatives
{
   public boolean updateAnglesGivenAngleDAB(double angleDABInRadians);
   public void computeMasterJointAngleGivenAngleABC(double angleABCInRadians);
   public void computeMasterJointAngleGivenAngleBCD(double angleBCDInRadians);
   public void computeMasterJointAngleGivenAngleCDA(double angleCDAInRadians);
   public boolean updateAnglesAndVelocitiesGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB);
   public boolean updateAnglesVelocitiesAndAccelerationsGivenAngleDAB(double angleDABInRadians, double angularVelocityDAB, double angularAccelerationDAB);

   public double getAngleDAB();
   public double getAngleABC();
   public double getAngleBCD();
   public double getAngleCDA();
   public double getAngleDtDAB();
   public double getAngleDtABC();
   public double getAngleDtBCD();
   public double getAngleDtCDA();
   public double getAngleDt2DAB();
   public double getAngleDt2ABC();
   public double getAngleDt2BCD();
   public double getAngleDt2CDA();
   public double getMinDAB();
   public double getMaxDAB();
   public double getAB();
   public double getBC();
   public double getCD();
   public double getDA();
}
