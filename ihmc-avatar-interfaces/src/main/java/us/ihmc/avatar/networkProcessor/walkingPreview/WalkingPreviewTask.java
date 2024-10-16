package us.ihmc.avatar.networkProcessor.walkingPreview;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commons.stateMachine.core.State;

public interface WalkingPreviewTask extends State
{
   default InverseDynamicsCommand<?> getOutput()
   {
      return null;
   }
}
