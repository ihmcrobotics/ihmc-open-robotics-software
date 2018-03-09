package us.ihmc.commonWalkingControlModules.controllerCore.command;

public enum ControllerCoreCommandType
{
   TASKSPACE, POINT, ORIENTATION, JOINTSPACE, MOMENTUM,
   PRIVILEGED_CONFIGURATION, PRIVILEGED_ACCELERATION, PRIVILEGED_VELOCITY,
   LIMIT_REDUCTION, JOINT_LIMIT_ENFORCEMENT,
   EXTERNAL_WRENCH, PLANE_CONTACT_STATE, CENTER_OF_PRESSURE,
   JOINT_ACCELERATION_INTEGRATION,
   COMMAND_LIST,
   VIRTUAL_WRENCH, VIRTUAL_FORCE, VIRTUAL_TORQUE, CONTROLLED_BODIES;

   private static final ControllerCoreCommandType[] inverseKinematicsCommands = { TASKSPACE, JOINTSPACE, MOMENTUM, PRIVILEGED_CONFIGURATION, PRIVILEGED_VELOCITY,
         LIMIT_REDUCTION, COMMAND_LIST };

   private static final ControllerCoreCommandType[] inverseDynamicsCommands = { TASKSPACE, JOINTSPACE, MOMENTUM, PRIVILEGED_CONFIGURATION, PRIVILEGED_ACCELERATION,
         LIMIT_REDUCTION, JOINT_LIMIT_ENFORCEMENT, EXTERNAL_WRENCH, PLANE_CONTACT_STATE, CENTER_OF_PRESSURE, JOINT_ACCELERATION_INTEGRATION, COMMAND_LIST };

   private static final ControllerCoreCommandType[] virtualModelControlCommands = { MOMENTUM, EXTERNAL_WRENCH, PLANE_CONTACT_STATE, VIRTUAL_WRENCH, VIRTUAL_FORCE,
         VIRTUAL_TORQUE, JOINTSPACE, CONTROLLED_BODIES, COMMAND_LIST };

   private static final ControllerCoreCommandType[] feedbackControlCommands = { TASKSPACE, POINT, ORIENTATION, JOINTSPACE, MOMENTUM, COMMAND_LIST };

   public static ControllerCoreCommandType[] getInverseKinematicsCommands()
   {
      return inverseKinematicsCommands;
   }

   public static ControllerCoreCommandType[] getInverseDynamicsCommands()
   {
      return inverseDynamicsCommands;
   }

   public static ControllerCoreCommandType[] getVirtualModelControlCommands()
   {
      return virtualModelControlCommands;
   }

   public static ControllerCoreCommandType[] getFeedbackControlCommands()
   {
      return feedbackControlCommands;
   }

}
