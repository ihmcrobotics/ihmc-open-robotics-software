package us.ihmc.acsell.hardware.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class AcsellAnkleFullComputation implements AcsellAnkleAngleCalculator
{	   
	   // Cable reduction of pulleys
	   private final double N;
	                                  	   
	   private final DenseMatrix64F Jit = new DenseMatrix64F(2, 2);
	   private final DenseMatrix64F Jt = new DenseMatrix64F(2, 2);
	   private final DenseMatrix64F J = new DenseMatrix64F(2, 2);
	   
	   private double q_AnkleX, q_AnkleY, q_rightActuator, q_leftActuator;
      private double sx, sy, cx, cy; 
	   private final DenseMatrix64F measuredJointVelocities = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F desiredJointAccelerations = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F measuredMotorVelocities = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F measuredMotorTorque = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F desiredJointTorque = new DenseMatrix64F(2,1);
	   
	   private double q_AnkleX_calc, q_AnkleY_calc, q_rightActuator_calc, q_leftActuator_calc;
	   private final DenseMatrix64F computedJointVelocities = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F computedMotorVelocities = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F computedMotorAccelerations = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F computedTauAnkle = new DenseMatrix64F(2,1);  
	   private final DenseMatrix64F computedTauDesiredActuators = new DenseMatrix64F(2,1);
	   
	   private final DenseMatrix64F Jdot = new DenseMatrix64F(2,2);
	   
	   //create some temporary matrices to help with some matrix functions
	   //these are created now as finals to help the garbage collector
	   private final DenseMatrix64F temp2by1 = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F temp2by1_2 = new DenseMatrix64F(2,1);

      private final AcsellAnkleSingleSidedComputation ankleRightCalculator;
      private final AcsellAnkleSingleSidedComputation ankleLeftCalculator;

   public AcsellAnkleFullComputation(AcsellAnkleKinematicParameters parameters, RobotSide side)
   {  
      switch (side)
      {
         case LEFT:
         {
            ankleRightCalculator = new AcsellAnkleSingleSidedComputation(parameters.getLeftAnkleRightParams());
            ankleLeftCalculator = new AcsellAnkleSingleSidedComputation(parameters.getLeftAnkleLeftParams());
            break;
         }
         case RIGHT:
         {
            ankleRightCalculator = new AcsellAnkleSingleSidedComputation(parameters.getRightAnkleRightParams());
            ankleLeftCalculator = new AcsellAnkleSingleSidedComputation(parameters.getRightAnkleLeftParams());
            break;            
         }
         default:
         {
            ankleRightCalculator = null;
            ankleLeftCalculator = null;
            break;
         }
      }
      
      N = parameters.getN();
   }
	   
   
   @Override
   public void updateAnkleState(AcsellActuatorState rightActuator, AcsellActuatorState leftActuator)
   {	   	   
	   q_AnkleX = AngleTools.trimAngleMinusPiToPi(rightActuator.getJointPosition());
	   q_AnkleY = AngleTools.trimAngleMinusPiToPi(leftActuator.getJointPosition());
	   measuredJointVelocities.set(0,rightActuator.getJointVelocity());
	   measuredJointVelocities.set(1,leftActuator.getJointVelocity());
	   
	   q_rightActuator = rightActuator.getMotorPosition();
	   q_leftActuator = leftActuator.getMotorPosition();
	   measuredMotorVelocities.set(0,rightActuator.getMotorVelocity());
	   measuredMotorVelocities.set(1,leftActuator.getMotorVelocity());
	   measuredMotorTorque.set(0,rightActuator.getMotorTorque());
	   measuredMotorTorque.set(1,leftActuator.getMotorTorque());	   
	   	   
		updateAnkleStateFromJointAngles(q_AnkleX, q_AnkleY);
   	computeJointTau();	   
   }
   
   @Override
   public void updateAnkleState(AcsellJointCommand ankleX, AcsellJointCommand ankleY)
   {	   	   
	   q_AnkleX = ankleX.getQ();
	   q_AnkleY = ankleY.getQ();
	   measuredJointVelocities.set(0,ankleX.getQd());
	   measuredJointVelocities.set(1,ankleY.getQd());

      q_rightActuator = ankleX.getMotorAngle(0);
      q_leftActuator = ankleX.getMotorAngle(1);   
	   desiredJointAccelerations.set(0,ankleX.getQdd_d());
	   desiredJointAccelerations.set(1,ankleY.getQdd_d());
	   desiredJointTorque.set(0,ankleX.getTauDesired());
	   desiredJointTorque.set(1,ankleY.getTauDesired());
	   
	   computeJacobiansAndDerivativesFromJointAngles(q_AnkleX, q_AnkleY);
	   computeActuatorQdd();
	   computeDesiredAcutatorTau();
   }
   
   @Override
   public void updateAnkleState(OneDoFJoint ankleX, OneDoFJoint ankleY)
   {	   	   
	   q_AnkleX = ankleX.getQ();
	   q_AnkleY = ankleY.getQ();
	   measuredJointVelocities.set(0,ankleX.getQd());
	   measuredJointVelocities.set(1,ankleY.getQd());
	   
	   desiredJointTorque.set(0,ankleX.getTau());
	   desiredJointTorque.set(1,ankleY.getTau());  
	   
	   updateAnkleStateFromJointAngles(q_AnkleX, q_AnkleY);
	   computeDesiredAcutatorTau();
   }
      
   private void updateAnkleStateFromJointAngles(double angleX, double angleY)
   {	   
      sx = Math.sin(angleX);
      sy = Math.sin(angleY);
      cx = Math.cos(angleX);
      cy = Math.cos(angleY);

      ankleRightCalculator.update(sx, sy, cx, cy);
	   ankleLeftCalculator.update(sx, sy, cx, cy);

      q_rightActuator_calc = ankleRightCalculator.getPulleyAngle()*N;
      q_leftActuator_calc = ankleLeftCalculator.getPulleyAngle()*N;

      ankleRightCalculator.getJacobianRow(0, J);
      ankleLeftCalculator.getJacobianRow(1, J);
      CommonOps.transpose(J, Jt);
      CommonOps.invert(Jt,Jit);  

      computeMotorVelocities();
   }
     
   @Override
   public double getQAnkleX()
   {
	   return q_AnkleX;
   }

   @Override
   public double getQAnkleY()
   {
      return q_AnkleY;
   }
   
   @Override
   public double getComputedQAnkleX()
   {
      return q_AnkleX_calc;
   }
   
   @Override
   public double getComputedQAnkleY()
   {
      return q_AnkleY_calc;
   }
   
   @Override
   public double getComputedQrightActuator()
   {
      return q_rightActuator_calc;
   }
   
   @Override
   public double getComputedQleftActuator()
   {
      return q_leftActuator_calc;
   }
   
   @Override
   public double getComputedQdAnkleX()
   {
      return computedJointVelocities.get(0);
   }

   @Override
   public double getComputedQdAnkleY()
   {
      return computedJointVelocities.get(1);
   }
   
   @Override
   public double getComputedQdRightActuator()
   {
      return computedMotorVelocities.get(0);
   }
   
   @Override
   public double getComputedQdLeftActuator()
   {
      return computedMotorVelocities.get(1);
   }
   
   @Override
   public double getComputedTauRightActuator()
   {
      return computedTauDesiredActuators.get(0);
   }
   

   @Override
   public double getComputedTauLeftActuator()
   {
      return computedTauDesiredActuators.get(1);
   }

   @Override
   public double getComputedTauAnkleX()
   {
      return computedTauAnkle.get(0);
   }
   @Override
   public double getComputedTauAnkleY()
   {
      return computedTauAnkle.get(1);
   }
      
   @Override
   public double getRatio()
   {
      return N;
   }
      
   private void computeJacobiansAndDerivativesFromJointAngles(double angleX, double angleY)
   {
      updateAnkleStateFromJointAngles(angleX, angleY);
      double xdot = measuredJointVelocities.get(0);
      double ydot = measuredJointVelocities.get(1);

      ankleRightCalculator.computeJacobianTimeDerivativeRow(xdot, ydot, getComputedQdRightActuator()/N);
      ankleLeftCalculator.computeJacobianTimeDerivativeRow(xdot, ydot, getComputedQdLeftActuator()/N);
      
      ankleRightCalculator.getJacobianTimeDerivativeRow(0, Jdot);
      ankleLeftCalculator.getJacobianTimeDerivativeRow(1, Jdot);      
   }
     
   private void computeMotorVelocities()
   {
	   CommonOps.mult(N, J,measuredJointVelocities,computedMotorVelocities);
   }
   
   private void computeJointVelocities()
   {
	   CommonOps.multTransA(1.0/N, Jit,measuredMotorVelocities,computedJointVelocities);
   }
   
   //Qdd_m = N*(Jdot*Qd_j + J*Qdd_j)
   private void computeActuatorQdd()
   {      
      CommonOps.mult(Jdot, measuredJointVelocities, temp2by1);
      CommonOps.mult(J, desiredJointAccelerations, temp2by1_2);
      CommonOps.add(temp2by1, temp2by1_2, computedMotorAccelerations);
      CommonOps.scale(N, computedMotorAccelerations);
   }
   
   // motorTorque = Jit * ankleTorques
   private void computeDesiredAcutatorTau()
   {
      CommonOps.mult(1.0/N,Jit, desiredJointTorque, computedTauDesiredActuators); 
   }
   
   private void computeJointTau()
   {
	   CommonOps.mult(N, Jt, measuredMotorTorque, computedTauAnkle); 
   }
   
   @Override
   public double getComputedMotorVelocityRight()
   {
      return computedMotorVelocities.get(0);
   }
   
   @Override
   public double getComputedMotorVelocityLeft()
   {
      return computedMotorVelocities.get(1);
   }
   
   @Override
   public double getComputedActuatorQddRight()
   {
      return computedMotorAccelerations.get(0);
   }
   @Override
   public double getComputedActuatorQddLeft()
   {
      return computedMotorAccelerations.get(1);
   }

   @Override @Deprecated
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft,
         double tauMeasureAnkleRight, double tauMeasureAnkleLeft)
   {
      // TODO Auto-generated method stub
      
   }



}
