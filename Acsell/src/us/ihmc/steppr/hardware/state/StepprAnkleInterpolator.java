package us.ihmc.steppr.hardware.state;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class StepprAnkleInterpolator implements StepprAnkleAngleCalculator
{

   private static final double px[] = { 0.000000000011833, 1.269023941053808, -1.269023941048548, 0.020516020869627, -0.020516021471959, 0.000000000606667,
         0.115495040111334, -0.523312217421704, 0.523312217621650, -0.115495040305941 };

   private static final double py[] = { 0.000022001174462, 0.463732921959109, 0.463732921783293, -0.048394601071517, -0.048394601065684, 0.111755077611979,
         -0.075022109801143, 0.057936370829754, 0.057936363252024, -0.075022107299457 };

   private static final double pJitX[] = { 1.272997427777619, 0.036001701491556, 0.001270562468774, 0.411410640321290, 0.628599891388498, -1.233612302811653,
         -0.128100680179965, 0.209457856510320, 0.014033597275913, -0.080071351885635 };

   private static final double pJitY[] = { 0.465442714987602, -0.079404498198185, 0.091522714637596, -0.239129834635615, 0.040327656123728, 0.141924052147264,
         -0.130148712431276, 0.342409832258836, -0.315199115178924, 0.111764662233772 };
   
   private static final double pM1[] = { 0.001637333497514, 2.363024415727514, 6.448426531047804, 0.209647991746190, -0.132578850323121, -0.102263276506052,
         -0.223803048871049, 0.764689039287297, 0.459191580065332, 0.349362854084994 };
   
   private static final double pM2[] = { 0.001637333371246, -2.363024413970044, 6.448426532362931, 0.209647993650533, -0.132578850320880, 0.102263276373644,
         0.223803036060187, 0.764689036580238, -0.459191581057666, 0.349362851648649 };

   private static final double N = 6.0; // Cable reduction of pulleys
   // this returns the value a 2D cubic polynomial with given parameters p
   // the m1,m2 inputs are the motor1 and 2 pulley angles
   private static double ScaledCubicApprox(double p[], double m1, double m2)
   {

      m1 /= N; //parameters were given for pulley angle not motor angle, corrected here
      m2 /= N; //parameters were given for pulley angle not motor angle, corrected here

      return CubicApprox(p, m1, m2);

   }

   private static double CubicApprox(double[] p, double m1, double m2)
   {
      double val = p[0] + p[1] * m1 + p[2] * m2 + p[3] * m1 * m1 + p[4] * m2 * m2 + p[5] * m1 * m2 + p[6] * m1 * m1 * m1 + p[7] * m1 * m1 * m2 + p[8] * m1 * m2
            * m2 + p[9] * m2 * m2 * m2;

      return val;
   }
   
   private static double ScaledCubicApproxDerivative_wrt_m1(double[] p, double m1, double m2)
   {
      m1 /= N;
      m2 /= N;
      double val = p[1] + 2*p[3] * m1 + p[5] * m2 + 3*p[6] * m1 * m1 + 2*p[7] * m1 * m2 + p[8] * m2 * m2;

      return val/N;
   }
   
   private static double ScaledCubicApproxDerivative_wrt_m2(double[] p, double m1, double m2)
   {
      m1 /= N;
      m2 /= N;
      double val = p[2] + 2*p[4] * m2 + p[5] * m1 + p[7] * m1 * m1 + 2*p[8] * m1 * m2 + 3*p[9] * m2 * m2;

      return val/N;
   }

   private static void JacobianInverseTranspose(DenseMatrix64F Jit, double m1, double m2)
   {

      Jit.set(0, 0, ScaledCubicApprox(pJitX, m1, m2)); //Jit11
      Jit.set(0, 1, ScaledCubicApprox(pJitY, m1, m2)); //Jit12
      Jit.set(1, 0, -ScaledCubicApprox(pJitX, m2, m1)); //Jit21
      Jit.set(1, 1, ScaledCubicApprox(pJitY, m2, m1)); //Jit22

   }
   
   
   private final DenseMatrix64F dJidq1 = new DenseMatrix64F(2,2);
   private final DenseMatrix64F dJidq2 = new DenseMatrix64F(2,2);
   private final DenseMatrix64F temp2by2 = new DenseMatrix64F(2,2);
   private final DenseMatrix64F Jdot = new DenseMatrix64F(2,2);
   private final DenseMatrix64F Jidot = new DenseMatrix64F(2,2);
   private void computeJacobiansAndDerivatives(double m1, double m2, double m1dot, double m2dot)
   {

      dJidq1.set(0, 0, ScaledCubicApproxDerivative_wrt_m1(pJitX, m1, m2)); //Jit11
      dJidq1.set(1, 0, ScaledCubicApproxDerivative_wrt_m1(pJitY, m1, m2)); //Jit12
      dJidq1.set(0, 1, -ScaledCubicApproxDerivative_wrt_m1(pJitX, m2, m1)); //Jit21
      dJidq1.set(1, 1, ScaledCubicApproxDerivative_wrt_m1(pJitY, m2, m1)); //Jit22     
      
      dJidq2.set(0, 0, ScaledCubicApproxDerivative_wrt_m2(pJitX, m1, m2)); //Jit11
      dJidq2.set(1, 0, ScaledCubicApproxDerivative_wrt_m2(pJitY, m1, m2)); //Jit12
      dJidq2.set(0, 1, -ScaledCubicApproxDerivative_wrt_m2(pJitX, m2, m1)); //Jit21
      dJidq2.set(1, 1, ScaledCubicApproxDerivative_wrt_m2(pJitY, m2, m1)); //Jit22
      
      computeJacobians(m1, m2);
      
      CommonOps.add(m1dot, dJidq1, m2dot, dJidq2, Jidot);
      
      CommonOps.mult(-1.0, J, Jidot, temp2by2);
      CommonOps.mult(temp2by2, J, Jdot);
      
   }

   private final DenseMatrix64F Jit = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F Jt = new DenseMatrix64F(2, 2);
   private final DenseMatrix64F J = new DenseMatrix64F(2, 2);
   private double qAnkleX, qAnkleY;
   private final DenseMatrix64F qdAnkle = new DenseMatrix64F(2,1);
   private final DenseMatrix64F ComputedMotorVelocities = new DenseMatrix64F(2,1);
   private final DenseMatrix64F ComputedTauDesiredActuators = new DenseMatrix64F(2,1);
   private final DenseMatrix64F ComputedTauAnkle = new DenseMatrix64F(2,1);  

   private final DenseMatrix64F MeasuredMotorVelocities = new DenseMatrix64F(2,1);
   private final DenseMatrix64F MeasuredMotorTorque = new DenseMatrix64F(2,1);
   private final DenseMatrix64F MeasuredJointVelocities = new DenseMatrix64F(2,1);

   @Override
   public void updateAnkleState(double motorAngleRight, double motorAngleLeft, double motorVelocityRight, double motorVelocityLeft,
         double tauMeasureAnkleRight, double tauMeasureAnkleLeft)
   {
      qAnkleX = ScaledCubicApprox(px, motorAngleRight, motorAngleLeft);
      qAnkleY = ScaledCubicApprox(py, motorAngleRight, motorAngleLeft);

      computeJacobians(motorAngleRight, motorAngleLeft);    

      MeasuredMotorVelocities.set(0,motorVelocityRight);
      MeasuredMotorVelocities.set(1,motorVelocityLeft);
      
      CommonOps.multTransA(1.0/N, Jit,MeasuredMotorVelocities,qdAnkle);
      
      MeasuredMotorTorque.set(0,tauMeasureAnkleRight);
      MeasuredMotorTorque.set(1,tauMeasureAnkleLeft);
      CommonOps.mult(N, Jt,MeasuredMotorTorque,ComputedTauAnkle); 
   }

   @Override
   public double getQAnkleX()
   {
      return qAnkleX;
   }

   @Override
   public double getQAnkleY()
   {
      return qAnkleY;
   }

   @Override
   public double getQdAnkleX()
   {
      return qdAnkle.get(0);
   }

   @Override
   public double getQdAnkleY()
   {
      return qdAnkle.get(1);
   }

   
   private final DenseMatrix64F tauDesiredAnkle = new DenseMatrix64F(2,1);
   // motorTorque = Jit * ankleTorques
   @Override
   public void calculateDesiredTau(double motorAngleRight, double motorAngleLeft, double tauDesiredAnkleX, double tauDesiredAnkleY)
   {
      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);
      tauDesiredAnkle.set(0,tauDesiredAnkleX);
      tauDesiredAnkle.set(1,tauDesiredAnkleY);
      CommonOps.mult(1.0/N,Jit,tauDesiredAnkle,ComputedTauDesiredActuators); 
   }

   
   @Override
   public double calculateRightMotorAngle(double ankleX, double ankleY)
   {
      return CubicApprox(pM1, ankleX, ankleY);
   }

   @Override
   public double calculateLeftMotorAngle(double ankleX, double ankleY)
   {
      return CubicApprox(pM2, ankleX, ankleY);
   }

   @Override
   public double getTauRightActuator()
   {
      return ComputedTauDesiredActuators.get(0);
   }
   

   @Override
   public double getTauLeftActuator()
   {
      return ComputedTauDesiredActuators.get(1);
   }

   @Override
   public double getTauAnkleX()
   {
      return ComputedTauAnkle.get(0);
   }
   @Override
   public double getTauAnkleY()
   {
      return ComputedTauAnkle.get(1);
   }
      
   @Override
   public double getRatio()
   {
      return N;
   }

   private void computeJacobians(double motorAngleRight, double motorAngleLeft)
   {
      JacobianInverseTranspose(Jit, motorAngleRight, motorAngleLeft);
      CommonOps.invert(Jit,Jt);
      CommonOps.transpose(Jt, J);      
   }
   
   @Override
   public void calculateActuatordQd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY)
   {
      computeJacobians(motorAngleRight, motorAngleLeft);
      MeasuredJointVelocities.set(0, qdAnkleX);
      MeasuredJointVelocities.set(1,qdAnkleY);
      CommonOps.mult(N, J,qdAnkle,ComputedMotorVelocities);
   }
   
   
   private final DenseMatrix64F temp2by1 = new DenseMatrix64F(2,1);
   private final DenseMatrix64F temp2by1_2 = new DenseMatrix64F(2,1);
   private final DenseMatrix64F MeasuredJointAccelerations = new DenseMatrix64F(2,1);
   private final DenseMatrix64F ComputedMotorAccelerations = new DenseMatrix64F(2,1);
   
   @Override
   public void calculateActuatordQdd(double motorAngleRight, double motorAngleLeft, double qdAnkleX, double qdAnkleY, double qddAnkleX, double qddAnkleY)
   {
      calculateActuatordQd(motorAngleRight,motorAngleLeft,qdAnkleX,qdAnkleY);
      computeJacobiansAndDerivatives(motorAngleRight, motorAngleLeft, this.getMotorVelocityLeft(), this.getMotorVelocityRight());
      
      MeasuredJointAccelerations.set(0,qddAnkleX);
      MeasuredJointAccelerations.set(1,qddAnkleY);
      CommonOps.mult(Jdot, MeasuredJointAccelerations, temp2by1);
      CommonOps.mult(J,MeasuredJointVelocities,temp2by1_2);
      CommonOps.add(N, temp2by1, N, temp2by1_2, ComputedMotorAccelerations);
      
   }
   
   
   @Override
   public double getMotorVelocityRight()
   {
      return ComputedMotorVelocities.get(0);
   }
   
   @Override
   public double getMotorVelocityLeft()
   {
      return ComputedMotorVelocities.get(1);
   }
   
   @Override
   public double getActuatorQddRight()
   {
      return ComputedMotorAccelerations.get(0);
   }
   @Override
   public double getActuatorQddLeft()
   {
      return ComputedMotorAccelerations.get(1);
   }

}
