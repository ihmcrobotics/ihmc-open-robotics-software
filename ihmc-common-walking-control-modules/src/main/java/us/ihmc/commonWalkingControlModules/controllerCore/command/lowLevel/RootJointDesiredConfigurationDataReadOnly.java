package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import controller_msgs.msg.dds.RobotDesiredConfigurationData;
import org.ejml.data.DMatrixRMaj;

public interface RootJointDesiredConfigurationDataReadOnly
{
   boolean hasDesiredConfiguration();

   boolean hasDesiredVelocity();

   boolean hasDesiredAcceleration();

   /**
    * Desired root joint position and orientation as a 7x1 matrix
    * Elements 0 - 3 are desired orientation of the root joint as a quaternion
    * Elements 4 - 6 are desired position of the root joint
    */
   DMatrixRMaj getDesiredConfiguration();

   /**
    * Desired root joint velocity as a 6x1 matrix
    * Elements 0 - 2 are desired angular velocity of the root joint
    * Elements 3 - 5 are desired linear velocity of the root joint
    */
   DMatrixRMaj getDesiredVelocity();

   /**
    * Desired root joint acceleration as a 6x1 matrix.
    * Elements 0 - 2 are desired angular acceleration of the root joint
    * Elements 3 - 5 are desired linear acceleration of the root joint
    */
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