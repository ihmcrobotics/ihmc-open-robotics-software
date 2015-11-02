package us.ihmc.acsell.hardware.state;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public interface AcsellAnkleAngleCalculator
{
   @Deprecated
   void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft, double tauMeasureAnkleRight,
         double tauMeasureAnkleLeft);
   
   public void updateAnkleState(AcsellActuatorState rightActuator, AcsellActuatorState leftActuator);
   
   public void updateAnkleState(AcsellJointCommand ankleX, AcsellJointCommand ankleY);
   
   public void updateAnkleState(OneDoFJoint ankleX, OneDoFJoint ankleY);
   
   public double getQAnkleX();

   public double getQAnkleY();

//   public double getQdAnkleX();

//   public double getQdAnkleY();

//   public void calculateDesiredTau(double motorAngleRight, double motorAngleLeft, double tauDesiredAnkleX, double tauDesiredAnkleY);

   public double getComputedTauRightActuator();

   public double getComputedTauLeftActuator();

   public double getRatio();

//   double calculateRightMotorAngle(double ankleX, double ankleY);

//   double calculateLeftMotorAngle(double ankleX, double ankleY);

   double getComputedTauAnkleX();

   double getComputedTauAnkleY();

   double getComputedMotorVelocityLeft();

   double getComputedMotorVelocityRight();

   //void calculateActuatordQd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY);

   //void calculateActuatordQdd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY, double qddAnkleX, double qddAnkleY);

   double getComputedActuatorQddLeft();

   double getComputedActuatorQddRight();

   double getComputedQAnkleX();

   double getComputedQAnkleY();

   double getComputedQrightActuator();

   double getComputedQleftActuator();

	double getComputedQdAnkleX();
	
	double getComputedQdAnkleY();
	
	double getComputedQdRightActuator();
	
	double getComputedQdLeftActuator();



}
