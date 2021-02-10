package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * This represents the commands that are submitted to the MPC core.
 * TODO extend settable
 */
public interface MPCCommand<T extends MPCCommand<T>>
{
   MPCCommandType getCommandType();
}
