package us.ihmc.avatar.kinematicsSimulation;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.stateMachine.core.State;

public interface KinematicsWalkingTask extends State
{
   default InverseDynamicsCommand<?> getOutput()
   {
      return null;
   }
}
