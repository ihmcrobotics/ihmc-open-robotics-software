package us.ihmc.avatar.networkProcessor.walkingPreview;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.robotics.taskExecutor.Task;

public interface WalkingPreviewTask extends Task
{
   default InverseDynamicsCommand<?> getOutput()
   {
      return null;
   }
}
