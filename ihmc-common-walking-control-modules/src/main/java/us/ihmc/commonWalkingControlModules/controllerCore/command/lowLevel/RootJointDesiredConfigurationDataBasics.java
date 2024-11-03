package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DMatrixRMaj;

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

   default void setDesiredConfiguration(DMatrixRMaj q)
   {
      setDesiredConfiguration(q, 0);
   }

   default void setDesiredVelocity(DMatrixRMaj qd)
   {
      setDesiredVelocity(qd, 0);
   }

   default void setDesiredAcceleration(DMatrixRMaj qdd)
   {
      setDesiredAcceleration(qdd, 0);
   }

   void setDesiredConfiguration(DMatrixRMaj q, int startIndex);

   void setDesiredVelocity(DMatrixRMaj qd, int startIndex);

   void setDesiredAcceleration(DMatrixRMaj qdd, int startIndex);
}