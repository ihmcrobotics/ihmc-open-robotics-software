package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DMatrixRMaj;

public interface RootJointDesiredConfigurationDataReadOnly
{
   boolean hasDesiredConfiguration();

   boolean hasDesiredVelocity();

   boolean hasDesiredAcceleration();

   DMatrixRMaj getDesiredConfiguration();

   DMatrixRMaj getDesiredVelocity();

   DMatrixRMaj getDesiredAcceleration();
}