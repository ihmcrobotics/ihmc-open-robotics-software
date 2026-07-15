package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import controller_msgs.RobotDesiredConfigurationData;
import org.ejml.data.DMatrixRMaj;

public interface RootJointDesiredConfigurationDataReadOnly
{
   boolean hasDesiredConfiguration();

   boolean hasDesiredVelocity();

   boolean hasDesiredAcceleration();

   DMatrixRMaj getDesiredConfiguration();

   DMatrixRMaj getDesiredVelocity();

   DMatrixRMaj getDesiredAcceleration();

   default void copyToMessage(RobotDesiredConfigurationData desiredConfigurationData)
   {
      desiredConfigurationData.setHasDesiredRootJointPositionData(hasDesiredConfiguration());
      if (hasDesiredConfiguration())
      {
         DMatrixRMaj q = getDesiredConfiguration();
         desiredConfigurationData.getDesiredRootJointOrientation().getQuaternion().set(q.get(0), q.get(1), q.get(2), q.get(3));
         desiredConfigurationData.getDesiredRootJointTranslation().getVector().set(q.get(4), q.get(5), q.get(6));
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointTranslation().getVector().setToZero();
         desiredConfigurationData.getDesiredRootJointOrientation().getQuaternion().setToZero();
      }

      desiredConfigurationData.setHasDesiredRootJointVelocityData(hasDesiredVelocity());
      if (hasDesiredVelocity())
      {
         DMatrixRMaj qd = getDesiredVelocity();
         desiredConfigurationData.getDesiredRootJointAngularVelocity().getVector().set(qd.get(0), qd.get(1), qd.get(2));
         desiredConfigurationData.getDesiredRootJointLinearVelocity().getVector().set(qd.get(3), qd.get(4), qd.get(5));
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointLinearVelocity().getVector().setToZero();
         desiredConfigurationData.getDesiredRootJointAngularVelocity().getVector().setToZero();
      }

      desiredConfigurationData.setHasDesiredRootJointAccelerationData(hasDesiredAcceleration());
      if (hasDesiredAcceleration())
      {
         DMatrixRMaj qdd = getDesiredAcceleration();
         desiredConfigurationData.getDesiredRootJointAngularAcceleration().getVector().set(qdd.get(0), qdd.get(1), qdd.get(2));
         desiredConfigurationData.getDesiredRootJointLinearAcceleration().getVector().set(qdd.get(3), qdd.get(4), qdd.get(5));
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointLinearAcceleration().getVector().setToZero();
         desiredConfigurationData.getDesiredRootJointAngularAcceleration().getVector().setToZero();
      }
   }
}
