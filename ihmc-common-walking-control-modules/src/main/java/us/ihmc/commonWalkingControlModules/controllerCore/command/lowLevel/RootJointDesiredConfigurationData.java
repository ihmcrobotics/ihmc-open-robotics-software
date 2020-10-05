package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;

public class RootJointDesiredConfigurationData implements RootJointDesiredConfigurationDataReadOnly, RootJointDesiredConfigurationDataBasics
{
   private final DMatrixRMaj desiredConfiguration = new DMatrixRMaj(7, 0);
   private final DMatrixRMaj desiredVelocity = new DMatrixRMaj(6, 0);
   private final DMatrixRMaj desiredAcceleration = new DMatrixRMaj(6, 0);

   public RootJointDesiredConfigurationData()
   {
      clear();
   }

   @Override
   public void clear()
   {
      desiredConfiguration.reshape(0, 0);
      desiredVelocity.reshape(0, 0);
      desiredAcceleration.reshape(0, 0);
   }

   public void set(RootJointDesiredConfigurationData other)
   {
      RootJointDesiredConfigurationDataBasics.super.set(other);
   }

   /**
    * Complete the information held in this using other. Does not overwrite the data already set in
    * this.
    */
   @Override
   public void completeWith(RootJointDesiredConfigurationDataReadOnly other)
   {
      if (!hasDesiredConfiguration())
         desiredConfiguration.set(other.getDesiredConfiguration());
      if (!hasDesiredVelocity())
         desiredVelocity.set(other.getDesiredVelocity());
      if (!hasDesiredAcceleration())
         desiredAcceleration.set(other.getDesiredAcceleration());
   }

   @Override
   public void setDesiredAccelerationFromJoint(FloatingJointBasics sixDoFJoint)
   {
      desiredAcceleration.reshape(6, 1);
      sixDoFJoint.getJointAcceleration(0, desiredAcceleration);
   }

   @Override
   public void setDesiredConfiguration(FrameQuaternionReadOnly orientation, FramePoint3DReadOnly position)
   {
      desiredConfiguration.reshape(7, 1);
      orientation.get(0, desiredConfiguration);
      position.get(4, desiredConfiguration);
   }

   @Override
   public void setDesiredVelocity(FrameVector3DReadOnly angularVelocity, FrameVector3DReadOnly linearVelocity)
   {
      desiredVelocity.reshape(6, 1);
      angularVelocity.get(0, desiredVelocity);
      linearVelocity.get(3, desiredVelocity);
   }

   @Override
   public void setDesiredAcceleration(FrameVector3DReadOnly angularAcceleration, FrameVector3DReadOnly linearAcceleration)
   {
      desiredAcceleration.reshape(6, 1);
      angularAcceleration.get(0, desiredAcceleration);
      linearAcceleration.get(3, desiredAcceleration);
   }

   @Override
   public void setDesiredConfiguration(DMatrixRMaj q, int startIndex)
   {
      desiredConfiguration.reshape(7, 1);
      CommonOps_DDRM.extract(q, startIndex, startIndex + 7, 0, 1, desiredConfiguration, 0, 0);
   }

   @Override
   public void setDesiredVelocity(DMatrixRMaj qd, int startIndex)
   {
      desiredVelocity.reshape(6, 1);
      CommonOps_DDRM.extract(qd, startIndex, startIndex + 6, 0, 1, desiredVelocity, 0, 0);
   }

   @Override
   public void setDesiredAcceleration(DMatrixRMaj qdd, int startIndex)
   {
      desiredAcceleration.reshape(6, 1);
      CommonOps_DDRM.extract(qdd, startIndex, startIndex + 6, 0, 1, desiredAcceleration, 0, 0);
   }

   @Override
   public boolean hasDesiredConfiguration()
   {
      return desiredConfiguration.getNumRows() != 0;
   }

   @Override
   public boolean hasDesiredVelocity()
   {
      return desiredVelocity.getNumRows() != 0;
   }

   @Override
   public boolean hasDesiredAcceleration()
   {
      return desiredAcceleration.getNumRows() != 0;
   }

   @Override
   public DMatrixRMaj getDesiredConfiguration()
   {
      return desiredConfiguration;
   }

   @Override
   public DMatrixRMaj getDesiredVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public DMatrixRMaj getDesiredAcceleration()
   {
      return desiredAcceleration;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (obj == this)
      {
         return true;
      }
      else if (obj instanceof RootJointDesiredConfigurationData)
      {
         RootJointDesiredConfigurationData other = (RootJointDesiredConfigurationData) obj;
         if (!MatrixTools.equals(desiredConfiguration, other.desiredConfiguration))
            return false;
         if (!MatrixTools.equals(desiredVelocity, other.desiredVelocity))
            return false;
         if (!MatrixTools.equals(desiredAcceleration, other.desiredAcceleration))
            return false;
         return true;
      }
      else
      {
         return false;
      }
   }
}
