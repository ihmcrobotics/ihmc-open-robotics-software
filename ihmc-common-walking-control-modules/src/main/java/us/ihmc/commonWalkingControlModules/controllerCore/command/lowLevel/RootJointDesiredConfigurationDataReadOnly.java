package us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel;

import org.ejml.data.DenseMatrix64F;

public interface RootJointDesiredConfigurationDataReadOnly
{

   boolean hasDesiredConfiguration();

   boolean hasDesiredVelocity();

   boolean hasDesiredAcceleration();

   DenseMatrix64F getDesiredConfiguration();

   DenseMatrix64F getDesiredVelocity();

   DenseMatrix64F getDesiredAcceleration();

}