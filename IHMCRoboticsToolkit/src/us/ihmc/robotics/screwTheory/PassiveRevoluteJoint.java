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
   public void getTauMatrix(DenseMatrix64F matrix)
   {
      throw new RuntimeException("Cannot set torque of a passive joint");  
   }

   @Override
   public void getVelocityMatrix(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set velocity of a passive joint");
   }

   @Override
   public void getDesiredAccelerationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setDesiredAccelerationToZero()
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setTorqueFromWrench(Wrench jointWrench)
   {
      throw new RuntimeException("Cannot set torque of a passive joint");
   }

   @Override
   public void setDesiredAcceleration(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setQ(double q)
   {
      throw new RuntimeException("Cannot set position of a passive joint");
   }

   @Override
   public void setQd(double qd)
   {
      throw new RuntimeException("Cannot set velocity of a passive joint");
   }

   @Override
   public void setQdd(double qdd)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setQddDesired(double qddDesired)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setTau(double tau)
   {
      throw new RuntimeException("Cannot set torque of a passive joint");  
   }

   @Override
   public void getConfigurationMatrix(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set position of a passive joint");
   }

   @Override
   public void setConfiguration(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set position of a passive joint");
   }

   @Override
   public void setVelocity(DenseMatrix64F matrix, int rowStart)
   {
      throw new RuntimeException("Cannot set velocity of a passive joint");
   }

   @Override
   public void setJointPositionVelocityAndAcceleration(InverseDynamicsJoint originalJoint)
   {
      throw new RuntimeException("Cannot set position, velocity, or acceleration of a passive joint");
   }

   @Override
   public void setQddDesired(InverseDynamicsJoint originalJoint)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setIntegrateDesiredAccelerations(boolean integrateDesiredAccelerations)
   {
      throw new RuntimeException("Cannot modify the acceleration of a passive joint");
   }

   @Override
   public boolean isPassiveJoint()
   {
      return true;
   }
}
