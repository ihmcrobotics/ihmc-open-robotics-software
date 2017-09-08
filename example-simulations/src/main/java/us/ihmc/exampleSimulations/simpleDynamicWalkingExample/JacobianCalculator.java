package us.ihmc.exampleSimulations.simpleDynamicWalkingExample;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.exampleSimulations.simpleDynamicWalkingExample.RobotParameters.LinkNames;
import us.ihmc.robotics.robotSide.RobotSide;

public class JacobianCalculator
{
   private Step7IDandSCSRobot_pinKnee rob;
   private DenseMatrix64F jacobian, angularVelocities, wrench;
   private DenseMatrix64F resultToPack = new DenseMatrix64F();

   private double l1 = RobotParameters.LENGTHS.get(LinkNames.UPPER_LINK);
   private double l2 = RobotParameters.LENGTHS.get(LinkNames.LOWER_LINK);
   private double j11, j12, j21, j22;
   private double sin1, sin12, cos1, cos12;
   private double dataArray[];
   
   private double wHip, wKnee;
   private double tauHip, tauKnee;
   

   public JacobianCalculator()
   {
   }

   public DenseMatrix64F calculateJacobian(RobotSide robotSide)
   {
      j11 = -l1 * sin1 - l2 * sin12;
      j12 = -l2*sin12;
      j21 = l1*cos1 +l2*cos12;
      j22 = l2*cos12;
      
      dataArray = new double[] { j11, j12, j21, j22, 1, 1 };
      jacobian = new DenseMatrix64F(3, 2, true, dataArray);
      
      return jacobian;
   }
   
   public void linearVelFromAngularVel()
   {
      angularVelocities = new DenseMatrix64F(2, 1, true, new double[]{wHip,wKnee});
      CommonOps.mult(jacobian, angularVelocities, resultToPack);
   }

   public void torquesFromWrench()
   {
      wrench = new DenseMatrix64F(2, 1, true, new double[]{tauHip,tauKnee});
      CommonOps.transpose(jacobian);
      CommonOps.mult(jacobian, wrench, resultToPack);
   }
   
   public void update(RobotSide robotSide) //should be called twice. Once per side.
   {
      sin1 = Math.sin(rob.getHipPitch(robotSide));
      sin12 = Math.sin(rob.getHipPitch(robotSide) + rob.getKneePitch(robotSide));
      cos1 = Math.cos(rob.getHipPitch(robotSide));
      cos12 = Math.cos(rob.getHipPitch(robotSide) + rob.getKneePitch(robotSide));
      
      wHip = rob.getKneeVelocity(robotSide);
      wKnee = rob.getHipVelocity(robotSide);
      
      tauHip = rob.getHipTau(robotSide);
      tauKnee = rob.getKneeTau(robotSide);
   }

}
