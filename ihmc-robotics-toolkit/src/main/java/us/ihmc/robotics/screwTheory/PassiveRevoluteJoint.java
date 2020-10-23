package us.ihmc.robotics.screwTheory;

import org.ejml.data.DMatrix;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;

public class PassiveRevoluteJoint extends RevoluteJoint
{
   private final boolean isPartOfClosedKinematicLoop;

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
   public PassiveRevoluteJoint(String name, RigidBodyBasics predecessor, RigidBodyTransform transformToParent, Vector3DReadOnly jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      super(name, predecessor, transformToParent, jointAxis);
      this.isPartOfClosedKinematicLoop = isPartOfClosedKinematicLoop;
   }

   /**
    * Torque on a passive joint is always zero
    */
   @Override
   public int getJointTau(int rowStart, DMatrix matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), 1, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(0, 0, 0);
      return rowStart + 1;
   }

   @Override
   public void setJointAccelerationToZero()
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   @Override
   public void setJointWrench(WrenchReadOnly jointWrench)
   {
      throw new RuntimeException("Cannot set torque of a passive joint");
   }

   @Override
   public int setJointAcceleration(int rowStart, DMatrix matrix)
   {
      throw new RuntimeException("Cannot set acceleration of a passive joint");
   }

   /** 
    * If part of a kinematic loop, this should only be called by that loop's calculator 
    */
   @Override
   public void setQ(double q)
   {
      super.setQ(q);
   }

   /** 
    * If part of a kinematic loop, this should only be called by that loop's calculator 
    */
   @Override
   public void setQd(double qd)
   {
      super.setQd(qd);
   }

   /** 
    * If part of a kinematic loop, this should only be called by that loop's calculator 
    */
   @Override
   public void setQdd(double qdd)
   {
      super.setQdd(qdd);
   }

   @Override
   public void setTau(double tau)
   {
      throw new RuntimeException("Cannot set torque of a passive joint");
   }

   @Override
   public int setJointConfiguration(int rowStart, DMatrix matrix)
   {
      throw new RuntimeException("Cannot set position of a passive joint");
   }

   @Override
   public int setJointVelocity(int rowStart, DMatrix matrix)
   {
      throw new RuntimeException("Cannot set velocity of a passive joint");
   }

   public boolean isPartOfClosedKinematicLoop()
   {
      return isPartOfClosedKinematicLoop;
   }
}
