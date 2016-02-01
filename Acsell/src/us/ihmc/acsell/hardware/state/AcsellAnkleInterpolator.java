package us.ihmc.acsell.hardware.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.acsell.hardware.command.AcsellJointCommand;
import us.ihmc.acsell.hardware.configuration.AcsellAnkleKinematicParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class AcsellAnkleInterpolator implements AcsellAnkleAngleCalculator
{
	   private final boolean USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES;// = false;//If false use joint
	   private final boolean COMPUTE_JACOBIAN_FROM_JOINT_ANGLES;// = true;
	   private final boolean COMPUTE_JACOBIAN_FROM_MOTOR_ANGLES;// = true;
	   
	   // Cable reduction of pulleys
	   private final double N;
	                                  
	   private final double[] px;
	   private final double[] py;
	   private final double[] p_m_JitX;
	   private final double[] p_m_JitY;
	   private final double[] p_j_JitX;
	   private final double[] p_j_JitY;
	   private final double[] pM1;
	   private final double[] pM2;
	   
	   private final DenseMatrix64F Jit = new DenseMatrix64F(2, 2);
	   private final DenseMatrix64F Jt = new DenseMatrix64F(2, 2);
	   private final DenseMatrix64F J = new DenseMatrix64F(2, 2);
	   
	   private double q_AnkleX, q_AnkleY, q_rightActuator, q_leftActuator;
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
	   
	   private final DenseMatrix64F dJidq1 = new DenseMatrix64F(2,2);
	   private final DenseMatrix64F dJidq2 = new DenseMatrix64F(2,2);
	   private final DenseMatrix64F Jdot = new DenseMatrix64F(2,2);
	   private final DenseMatrix64F Jidot = new DenseMatrix64F(2,2);
	   
	   //create some temporary matrices to help with some matrix functions
	   //these are created now as finals to help the garbage collector
	   private final DenseMatrix64F temp2by1 = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F temp2by1_2 = new DenseMatrix64F(2,1);
	   private final DenseMatrix64F temp2by2 = new DenseMatrix64F(2,2);

   public AcsellAnkleInterpolator(AcsellAnkleKinematicParameters parameters)
   {
      N = parameters.getN();
      
      px = parameters.getXParams();
      py = parameters.getYParams();
      p_m_JitX = parameters.getJITX_FromMotorParams();
      p_m_JitY = parameters.getJITY_FromMotorParams();
      p_j_JitX = parameters.getJITX_FromJointParams();
      p_j_JitY = parameters.getJITY_FromJointParams();
      pM1 = parameters.getM1Params();
      pM2 = parameters.getM2Params();
      
      USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES = parameters.useJacobianComputedFromMotors();
      COMPUTE_JACOBIAN_FROM_JOINT_ANGLES = parameters.isJacobianFromJointAnglesComputationPerformed();
      COMPUTE_JACOBIAN_FROM_MOTOR_ANGLES = parameters.isJacobianFromMotorAnglesComputationPerformed();
  
   }
	   
   // this returns the value a 2D cubic polynomial with given parameters p
   // the m1,m2 inputs are the motor1 and motor2 pulley angles
   private double scaledDownCubicApprox(double p[], double m1, double m2)
   {

      m1 /= N; //parameters were given for pulley angle not motor angle, corrected here
      m2 /= N; //parameters were given for pulley angle not motor angle, corrected here

      return cubicApprox(p, m1, m2);

   }

   private double cubicApprox(double[] p, double m1, double m2)
   {
      double val = p[0] + p[1] * m1 + p[2] * m2 + p[3] * m1 * m1 + p[4] * m2 * m2 + p[5] * m1 * m2 + p[6] * m1 * m1 * m1 + p[7] * m1 * m1 * m2 + p[8] * m1 * m2
            * m2 + p[9] * m2 * m2 * m2;

      return val;
   }
   
   private double scaledCubicApproxDerivative_wrt_m1(double[] p, double m1, double m2)
   {
      m1 /= N;
      m2 /= N;
      double val = p[1] + 2*p[3] * m1 + p[5] * m2 + 3*p[6] * m1 * m1 + 2*p[7] * m1 * m2 + p[8] * m2 * m2;

      return val/N;
   }
   
   private double scaledCubicApproxDerivative_wrt_m2(double[] p, double m1, double m2)
   {
      m1 /= N;
      m2 /= N;
      double val = p[2] + 2*p[4] * m2 + p[5] * m1 + p[7] * m1 * m1 + 2*p[8] * m1 * m2 + 3*p[9] * m2 * m2;

      return val/N;
   }

   private void computeSymmetricLinkageJacobianInverseTransposeFromMotorAngles(DenseMatrix64F Jit, double m1, double m2)
   {

      Jit.set(0, 0, scaledDownCubicApprox(p_m_JitX, m1, m2)); //Jit11
      Jit.set(0, 1, scaledDownCubicApprox(p_m_JitY, m1, m2)); //Jit12
      Jit.set(1, 0, -scaledDownCubicApprox(p_m_JitX, m2, m1)); //Jit21
      Jit.set(1, 1, scaledDownCubicApprox(p_m_JitY, m2, m1)); //Jit22

   }
   
   private void computeSymmetricLinkageJacobianInverseTransposeFromJointAngles(DenseMatrix64F Jit, double q_x, double q_y)
   {

      Jit.set(0, 0, cubicApprox(p_j_JitX, q_x, q_y)); //Jit11
      Jit.set(0, 1, cubicApprox(p_j_JitY, q_x, q_y)); //Jit12
      Jit.set(1, 0, -cubicApprox(p_j_JitX, q_x, q_y)); //Jit21
      Jit.set(1, 1, cubicApprox(p_j_JitY, q_x, q_y)); //Jit22

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
	   
	   if (COMPUTE_JACOBIAN_FROM_JOINT_ANGLES || !USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {		   
		   updateAnkleStateFromJointAngles();
		   
	       if(!USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	       {
			   //update torque
	    	   computeJointTau();
	       }
	   }
	   if (COMPUTE_JACOBIAN_FROM_MOTOR_ANGLES || USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {
		   updateAnkleStateFromMotorAngles();
		   
		   if(USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
		   {
			   //update torque
			   computeJointTau();
		   }
	   }
   }
   
   @Override
   public void updateAnkleState(AcsellJointCommand ankleX, AcsellJointCommand ankleY)
   {	   	   
	   q_AnkleX = ankleX.getQ();
	   q_AnkleY = ankleY.getQ();
	   measuredJointVelocities.set(0,ankleX.getQd());
	   measuredJointVelocities.set(1,ankleY.getQd());
	   desiredJointAccelerations.set(0,ankleX.getQdd_d());
	   desiredJointAccelerations.set(1,ankleY.getQdd_d());
	   desiredJointTorque.set(0,ankleX.getTauDesired());
	   desiredJointTorque.set(1,ankleY.getTauDesired());
	   
	   q_rightActuator = ankleX.getMotorAngle(0);
	   q_leftActuator = ankleX.getMotorAngle(1);   
	   
	   if (COMPUTE_JACOBIAN_FROM_JOINT_ANGLES || !USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {		   
		   computeJacobiansAndDerivativesFromMotorAngles(q_rightActuator, q_leftActuator); //TODO: Make joint angle based function
		   
	       if(!USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	       {
			   //update torque
	    	   computeActuatorQdd();
	    	   computeDesiredAcutatorTau();
	       }
	   }
	   if (COMPUTE_JACOBIAN_FROM_MOTOR_ANGLES || USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {
		   computeJacobiansAndDerivativesFromMotorAngles(q_rightActuator, q_leftActuator);
		   
		   if(USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
		   {
			   //update torque
	    	   computeActuatorQdd();
	    	   computeDesiredAcutatorTau();			   
		   }
	   }
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
	   
	   if (COMPUTE_JACOBIAN_FROM_JOINT_ANGLES || !USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {		   
		   computeJacobiansFromJointAngles(q_AnkleX, q_AnkleY);
		   
	       if(!USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	       {
			   //update torque
	    	   computeMotorVelocities();
	    	   computeDesiredAcutatorTau();
	       }
	   }
	   if (COMPUTE_JACOBIAN_FROM_MOTOR_ANGLES || USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
	   {
		   q_rightActuator_calc = computeRightMotorAngle(q_AnkleX, q_AnkleY);
		   q_leftActuator_calc = computeLeftMotorAngle(q_AnkleX, q_AnkleY);
		   computeJacobiansFromMotorAngles(q_rightActuator_calc, q_leftActuator_calc);
		   
		   if(USE_JACOBIAN_COMPUTED_FROM_MOTOR_ANGLES)
		   {
			   //update torque
	    	   computeMotorVelocities();
	    	   computeDesiredAcutatorTau();			   
		   }
	   }
   }
      
   private void updateAnkleStateFromJointAngles()
   {	   
	   q_rightActuator_calc = computeRightMotorAngle(q_AnkleX, q_AnkleY);
	   q_leftActuator_calc = computeLeftMotorAngle(q_AnkleX, q_AnkleY);
	   	   
	   computeJacobiansFromJointAngles(q_AnkleX, q_AnkleY);
	   computeMotorVelocities();
   }
      
   private void updateAnkleStateFromMotorAngles()
   {	   
	   q_AnkleX_calc = scaledDownCubicApprox(px, q_rightActuator, q_leftActuator);
	   q_AnkleY_calc = scaledDownCubicApprox(py, q_rightActuator, q_leftActuator);
	   	   
	   computeJacobiansFromMotorAngles(q_rightActuator, q_leftActuator);
	   computeJointVelocities();
   }

   @Override @Deprecated
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft,
         double tauMeasureAnkleRight, double tauMeasureAnkleLeft)
   {
      q_AnkleX = scaledDownCubicApprox(px, motorAngleRight, motorAngleLeft);
      q_AnkleY = scaledDownCubicApprox(py, motorAngleRight, motorAngleLeft);

      computeJacobiansFromMotorAngles(motorAngleRight, motorAngleLeft);    

      measuredMotorVelocities.set(0,motorVelocityRight);
      measuredMotorVelocities.set(1,motorVelocityLeft);
      
      CommonOps.multTransA(1.0/N, Jit,measuredMotorVelocities,computedJointVelocities);
      
      measuredMotorTorque.set(0,tauMeasureAnkleRight);
      measuredMotorTorque.set(1,tauMeasureAnkleLeft);
      CommonOps.mult(N, Jt,measuredMotorTorque,computedTauAnkle); 
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

   private void computeJacobiansFromMotorAngles(double motorAngleRight, double motorAngleLeft)
   {
      computeSymmetricLinkageJacobianInverseTransposeFromMotorAngles(Jit, motorAngleRight, motorAngleLeft);
      CommonOps.invert(Jit,Jt);
      CommonOps.transpose(Jt, J);      
   }
   
   private void computeJacobiansFromJointAngles(double q_x, double q_y)
   {
      computeSymmetricLinkageJacobianInverseTransposeFromJointAngles(Jit, q_x, q_y);
      CommonOps.invert(Jit,Jt);
      CommonOps.transpose(Jt, J);      
   }
   
   private void computeJacobiansAndDerivativesFromMotorAngles(double motorAngleRight, double motorAngleLeft)
   {
	  computeJacobiansFromMotorAngles(motorAngleRight, motorAngleLeft);
	  computeMotorVelocities();

      dJidq1.set(0, 0, scaledCubicApproxDerivative_wrt_m1(p_m_JitX, motorAngleRight, motorAngleLeft)); //Jit11
      dJidq1.set(1, 0, scaledCubicApproxDerivative_wrt_m1(p_m_JitY, motorAngleRight, motorAngleLeft)); //Jit12
      dJidq1.set(0, 1, -scaledCubicApproxDerivative_wrt_m1(p_m_JitX, motorAngleLeft, motorAngleRight)); //Jit21
      dJidq1.set(1, 1, scaledCubicApproxDerivative_wrt_m1(p_m_JitY, motorAngleLeft, motorAngleRight)); //Jit22     
      
      dJidq2.set(0, 0, scaledCubicApproxDerivative_wrt_m2(p_m_JitX, motorAngleRight, motorAngleLeft)); //Jit11
      dJidq2.set(1, 0, scaledCubicApproxDerivative_wrt_m2(p_m_JitY, motorAngleRight, motorAngleLeft)); //Jit12
      dJidq2.set(0, 1, -scaledCubicApproxDerivative_wrt_m2(p_m_JitX, motorAngleLeft, motorAngleRight)); //Jit21
      dJidq2.set(1, 1, scaledCubicApproxDerivative_wrt_m2(p_m_JitY, motorAngleLeft, motorAngleRight)); //Jit22
            
      CommonOps.add(this.getComputedMotorVelocityRight(), dJidq1, this.getComputedMotorVelocityLeft(), dJidq2, Jidot);
      
      CommonOps.mult(-1.0, J, Jidot, temp2by2);
      CommonOps.mult(temp2by2, J, Jdot);
      
   }
   
   private double computeRightMotorAngle(double ankleX, double ankleY)
   {
      return cubicApprox(pM1, ankleX, ankleY);
   }

   private double computeLeftMotorAngle(double ankleX, double ankleY)
   {
      return cubicApprox(pM2, ankleX, ankleY);
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
      CommonOps.add(N, temp2by1, N, temp2by1_2, computedMotorAccelerations);      
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



}
