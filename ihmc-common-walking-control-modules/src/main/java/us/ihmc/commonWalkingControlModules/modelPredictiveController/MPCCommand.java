package us.ihmc.commonWalkingControlModules.modelPredictiveController;

// TODO extend settable
public interface MPCCommand<T extends MPCCommand<T>>
{
   MPCCommandType getCommandType();
}
