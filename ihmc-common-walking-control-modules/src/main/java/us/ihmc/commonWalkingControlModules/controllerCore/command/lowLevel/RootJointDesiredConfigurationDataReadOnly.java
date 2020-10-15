package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import controller_msgs.msg.dds.RobotDesiredConfigurationData;
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
      if(hasDesiredConfiguration())
      {
         desiredConfigurationData.getDesiredRootJointOrientation().set(0, getDesiredConfiguration());
         desiredConfigurationData.getDesiredRootJointTranslation().set(4, getDesiredConfiguration());
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointTranslation().setToZero();
         desiredConfigurationData.getDesiredRootJointOrientation().setToZero();
      }

      desiredConfigurationData.setHasDesiredRootJointVelocityData(hasDesiredVelocity());
      if(hasDesiredVelocity())
      {
         desiredConfigurationData.getDesiredRootJointAngularVelocity().set(0, getDesiredVelocity());
         desiredConfigurationData.getDesiredRootJointLinearVelocity().set(3, getDesiredVelocity());
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointLinearVelocity().setToZero();
         desiredConfigurationData.getDesiredRootJointAngularVelocity().setToZero();
      }

      desiredConfigurationData.setHasDesiredRootJointAccelerationData(hasDesiredAcceleration());
      if(hasDesiredAcceleration())
      {
         desiredConfigurationData.getDesiredRootJointAngularAcceleration().set(0, getDesiredAcceleration());
         desiredConfigurationData.getDesiredRootJointLinearAcceleration().set(3, getDesiredAcceleration());
      }
      else
      {
         desiredConfigurationData.getDesiredRootJointLinearAcceleration().setToZero();
         desiredConfigurationData.getDesiredRootJointAngularAcceleration().setToZero();
      }
   }
}