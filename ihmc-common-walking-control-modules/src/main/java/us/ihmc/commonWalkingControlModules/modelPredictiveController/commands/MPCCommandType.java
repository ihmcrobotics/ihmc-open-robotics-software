package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Enum specifying all the input types to the MPC core.
 */
public enum MPCCommandType
{
   VALUE, CONTINUITY, LIST, RHO_VALUE, RHO_BOUND, NORMAL_FORCE_BOUND, VRP_TRACKING, ORIENTATION_TRAJECTORY, ORIENTATION_VALUE, DIRECT_ORIENTATION_VALUE, ORIENTATION_CONTINUITY,
   FORCE_MINIMIZATION
}
