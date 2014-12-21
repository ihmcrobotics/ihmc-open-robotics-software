package us.ihmc.steppr.hardware.state;

public interface StepprAnkleAngleCalculator
{
   void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft, double tauMeasureAnkleRight,
         double tauMeasureAnkleLeft);

   public double getQAnkleX();

   public double getQAnkleY();

   public double getQdAnkleX();

   public double getQdAnkleY();

   public void calculateDesiredTau(double motorAngleRight, double motorAngleLeft, double tauDesiredAnkleX, double tauDesiredAnkleY);

   public double getTauRightActuator();

   public double getTauLeftActuator();

   public double getRatio();

   double calculateRightMotorAngle(double ankleX, double ankleY);

   double calculateLeftMotorAngle(double ankleX, double ankleY);

   double getTauAnkleX();

   double getTauAnkleY();

   double getMotorVelocityLeft();

   double getMotorVelocityRight();

   void calculateDesiredQd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY);

}
