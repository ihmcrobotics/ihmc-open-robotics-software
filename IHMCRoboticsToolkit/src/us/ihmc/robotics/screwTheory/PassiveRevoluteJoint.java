package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PassiveRevoluteJoint extends RevoluteJoint
{

   /**
    * In this type of joint:
    * 
    *    1)  You can NOT set any inputs (Q, Qd, or Tau) --> because they are not actuated 
    *    joints, in other words they cannot be controlled because their motion is determined 
    *    by that of another non-passive joint.
    *    
    *    2) getTau() should always return a zero because, since the joint is NOT actuated,
    *    there is no torque.
    */

   public PassiveRevoluteJoint(String name, RigidBody predecessor, ReferenceFrame beforeJointFrame, FrameVector jointAxis)
   {
      super(name, predecessor, beforeJointFrame, jointAxis);
   }

   @Override
   public void packTauMatrix(DenseMatrix64F matrix)
   {
      throw new RuntimeException("Cannot set Tau on passive joint");  
   }

   @Override
   public void packVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void packDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void setDesiredAccelerationToZero()
   {

   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {

   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void setQ(double q)
   {

   }

   @Override
   public void setQd(double qd)
   {

   }

   @Override
   public void setQdd(double qdd)
   {

   }

   @Override
   public void setQddDesired(double qddDesired)
   {

   }

   @Override
   public void setTau(double tau)
   {

   }

   @Override
   public void packConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {

   }

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {

   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {

   }

   @Override
   public void setJointLimitLower(double jointLimitLower)
   {

   }

   @Override
   public void setJointLimitUpper(double jointLimitUpper)
   {

   }

   @Override
   public void setEffortLimit(double effortLimit)
   {

   }

   @Override
   //   public double getEffortLimit()  //TODO does this make sense?
   //   {
   //     
   //   }

   public void setIntegrateDesiredAccelerations(boolean integrateDesiredAccelerations)
   {

   }

}
