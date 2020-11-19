package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.MPCCommandType;

// TODO extend settable
public interface MPCCommand<T extends MPCCommand<T>>
{
   MPCCommandType getCommandType();
}
