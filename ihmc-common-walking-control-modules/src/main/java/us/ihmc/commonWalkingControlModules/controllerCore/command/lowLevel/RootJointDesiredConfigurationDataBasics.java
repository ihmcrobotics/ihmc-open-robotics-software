package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;

public interface RootJointDesiredConfigurationDataBasics extends RootJointDesiredConfigurationDataReadOnly
{
   void clear();

   default void set(RootJointDesiredConfigurationDataReadOnly other)
   {
      clear();
      if (other.hasDesiredConfiguration())
         setDesiredConfiguration(other.getDesiredConfiguration());
      if (other.hasDesiredVelocity())
         setDesiredVelocity(other.getDesiredVelocity());
      if (other.hasDesiredAcceleration())
         setDesiredAcceleration(other.getDesiredAcceleration());
   }

   void completeWith(RootJointDesiredConfigurationDataReadOnly other);

   void setDesiredAccelerationFromJoint(FloatingJointBasics sixDoFJoint);

   void setDesiredConfiguration(FrameQuaternionReadOnly orientation, FramePoint3DReadOnly position);

   void setDesiredVelocity(FrameVector3DReadOnly angularVelocity, FrameVector3DReadOnly linearVelocity);

   void setDesiredAcceleration(FrameVector3DReadOnly angularAcceleration, FrameVector3DReadOnly linearAcceleration);

   default void setDesiredConfiguration(DenseMatrix64F q)
   {
      setDesiredConfiguration(q, 0);
   }

   default void setDesiredVelocity(DenseMatrix64F qd)
   {
      setDesiredVelocity(qd, 0);
   }

   default void setDesiredAcceleration(DenseMatrix64F qdd)
   {
      setDesiredAcceleration(qdd, 0);
   }

   void setDesiredConfiguration(DenseMatrix64F q, int startIndex);

   void setDesiredVelocity(DenseMatrix64F qd, int startIndex);

   void setDesiredAcceleration(DenseMatrix64F qdd, int startIndex);
}