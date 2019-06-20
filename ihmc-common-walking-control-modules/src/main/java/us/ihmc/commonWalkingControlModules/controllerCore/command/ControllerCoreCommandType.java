package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseKinematicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyVirtualModelControlSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;

public enum ControllerCoreCommandType
{
   /**
    * Represents a command for the {@link WholeBodyControllerCore} given in taskspace. This could be a {@link SpatialVelocityCommand} for the
    * {@link WholeBodyInverseKinematicsSolver}, a {@link SpatialAccelerationCommand} for the {@link WholeBodyInverseDynamicsSolver}, or a
    * {@link SpatialFeedbackControlCommand} for the {@link WholeBodyFeedbackController}.
    */
   TASKSPACE,

   /**
    * Represents a {@link PointFeedbackControlCommand} to try and achieve a certain linear position. This is submitted to the {@link WholeBodyFeedbackController}.
    */
   POINT,

   /**
    * Represents a {@link OrientationFeedbackControlCommand} to try and achieve a certain rotational orientation. This is submitted to the
    * {@link WholeBodyFeedbackController}.
    */
   ORIENTATION,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} given in jointspace. This could be a {@link JointspaceVelocityCommand} for the
    * {@link WholeBodyInverseKinematicsSolver}, a {@link JointspaceAccelerationCommand} for the {@link WholeBodyInverseDynamicsSolver}, a
    * {@link JointTorqueCommand} for the {@link WholeBodyVirtualModelControlSolver}, or a {@link SpatialFeedbackControlCommand} for the
    * {@link WholeBodyFeedbackController}.
    */
   JOINTSPACE,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} given for the center of mass. This could be a {@link MomentumCommand} for the
    * {@link WholeBodyInverseKinematicsSolver}, a {@link MomentumRateCommand} for the {@link WholeBodyInverseDynamicsSolver} or the
    * {@link WholeBodyVirtualModelControlSolver}, or a {@link CenterOfMassFeedbackControlCommand} for the {@link WholeBodyFeedbackController}.
    */
   MOMENTUM,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a desired privileged configuration. This is a configuration set for the robot to
    * help the {@link WholeBodyInverseKinematicsSolver} and {@link WholeBodyInverseDynamicsSolver} avoid singularities in their solutions. It is then used in an
    * internal feedback controller to determine a desired jointspace velocity or acceleration, respectively, in the null-space for the optimization modules.
    */
   PRIVILEGED_CONFIGURATION,

   /**
    * Represents a command for the {@link WholeBodyInverseDynamicsSolver} to specify a desired privileged joint space of a joint or joints. This is an
    * acceleration in the {@link WholeBodyInverseDynamicsSolver}, a velocity in the {@link WholeBodyInverseKinematicsSolver}, and will be a torque in the
    * {@link WholeBodyVirtualModelControlSolver}. This is then used in the task null-space to help the solution avoid singularities.
    */
   PRIVILEGED_JOINTSPACE_COMMAND,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a desired joint limit reduction using the {@link JointLimitReductionCommand}
    * for the {@link WholeBodyInverseKinematicsSolver} and {@link WholeBodyInverseDynamicsSolver}.
    */
   LIMIT_REDUCTION,

   /**
    * Represents a command for the {@link WholeBodyInverseDynamicsSolver} to specify a desired joint limit enforcement using the
    * {@link JointLimitEnforcementMethodCommand} so that the {@link WholeBodyInverseDynamicsSolver} respects the joint limits of the joints specified in the
    * {@link JointLimitEnforcementMethodCommand} using the {@link JointLimitEnforcement} method.
    */
   JOINT_LIMIT_ENFORCEMENT,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a desired external wrench using the {@link ExternalWrenchCommand}
    * for the {@link WholeBodyInverseDynamicsSolver} and {@link WholeBodyVirtualModelControlSolver}.
    */
   EXTERNAL_WRENCH,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a plane's contact state using the {@link PlaneContactStateCommand}
    * for the {@link WholeBodyInverseDynamicsSolver} and {@link WholeBodyVirtualModelControlSolver}.
    */
   PLANE_CONTACT_STATE,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a desired Center of Pressure for the reaction force optimization using the
    * {@link CenterOfPressureCommand} for the {@link WholeBodyInverseDynamicsSolver}.
    */
   CENTER_OF_PRESSURE,

   /**
    * Represents a command for the {@link WholeBodyControllerCore} to specify a the desired joint acceleration integration approach for the solution using the
    * {@link JointAccelerationIntegrationCommand} for the {@link WholeBodyInverseDynamicsSolver}.
    */
   JOINT_ACCELERATION_INTEGRATION,

   /**
    * Represents a list of commands for the {@link WholeBodyControllerCore}. This can be a {@link InverseKinematicsCommandList} for the
    * {@link WholeBodyInverseKinematicsSolver}, a {@link InverseDynamicsCommandList} for the {@link WholeBodyInverseDynamicsSolver}, a
    * {@link VirtualModelControlCommandList} for the {@link WholeBodyVirtualModelControlSolver}, or a {@link FeedbackControlCommandList} for the
    * {@link WholeBodyFeedbackController}..
    */
   COMMAND_LIST,

   /**
    * Represents a {@link VirtualWrenchCommand} for the {@link WholeBodyVirtualModelControlSolver}.
    * // TODO make this a TASKSPACE command.
    */
   VIRTUAL_WRENCH,

   /**
    * Represents a {@link VirtualForceCommand} for the {@link WholeBodyVirtualModelControlSolver}.
    * // TODO make this a TASKSPACE command.
    */
   VIRTUAL_FORCE,

   /**
    * Represents a {@link VirtualTorqueCommand} for the {@link WholeBodyVirtualModelControlSolver}.
    * // TODO make this a TASKSPACE command.
    */
   VIRTUAL_TORQUE,

   /**
    * An objective for the wrench exerted by a body in contact {@link WholeBodyInverseDynamicsSolver}. This is different from the
    * {@link #EXTERNAL_WRENCH} command in that it will be added to the optimization. E.g. this can be used to request that the
    * inverse dynamics optimization try to put a certain force on one foot or to constrain the foot wrench.
    */
   CONTACT_WRENCH,

   /**
    * Represents a command that modifies the optimization settings in the controller core. By default these settings are defined
    * in the {@link ControllerCoreOptimizationSettings}. This command allows changing some of the optimization settings such as
    * bounds on rhos online using the command API.
    */
   OPTIMIZATION_SETTINGS;

   private static final ControllerCoreCommandType[] inverseKinematicsCommands = {TASKSPACE, JOINTSPACE, MOMENTUM, PRIVILEGED_CONFIGURATION,
         PRIVILEGED_JOINTSPACE_COMMAND, LIMIT_REDUCTION, COMMAND_LIST};

   private static final ControllerCoreCommandType[] inverseDynamicsCommands = {TASKSPACE, JOINTSPACE, MOMENTUM, PRIVILEGED_CONFIGURATION,
         PRIVILEGED_JOINTSPACE_COMMAND, LIMIT_REDUCTION, JOINT_LIMIT_ENFORCEMENT, EXTERNAL_WRENCH, PLANE_CONTACT_STATE, CENTER_OF_PRESSURE,
         JOINT_ACCELERATION_INTEGRATION, CONTACT_WRENCH, COMMAND_LIST, OPTIMIZATION_SETTINGS};

   private static final ControllerCoreCommandType[] virtualModelControlCommands = {MOMENTUM, EXTERNAL_WRENCH, PLANE_CONTACT_STATE, VIRTUAL_WRENCH,
         VIRTUAL_FORCE, VIRTUAL_TORQUE, JOINTSPACE, JOINT_LIMIT_ENFORCEMENT, COMMAND_LIST};

   private static final ControllerCoreCommandType[] feedbackControlCommands = {TASKSPACE, POINT, ORIENTATION, JOINTSPACE, MOMENTUM, COMMAND_LIST};

   /**
    * Gets the list of available commands for the {@link WholeBodyInverseKinematicsSolver}.
    */
   public static ControllerCoreCommandType[] getInverseKinematicsCommands()
   {
      return inverseKinematicsCommands;
   }

   /**
    * Gets the list of available commands for the {@link WholeBodyInverseDynamicsSolver}.
    */
   public static ControllerCoreCommandType[] getInverseDynamicsCommands()
   {
      return inverseDynamicsCommands;
   }

   /**
    * Gets the list of available commands for the {@link WholeBodyVirtualModelControlSolver}.
    */
   public static ControllerCoreCommandType[] getVirtualModelControlCommands()
   {
      return virtualModelControlCommands;
   }

   /**
    * Gets the list of available commands for the {@link WholeBodyFeedbackController}.
    */
   public static ControllerCoreCommandType[] getFeedbackControlCommands()
   {
      return feedbackControlCommands;
   }

   /**
    * Polls whether the command {@code commandType} is an available command for the {@link WholeBodyInverseKinematicsSolver}.
    */
   public static boolean isInverseKinematicsCommand(ControllerCoreCommandType commandType)
   {
      for (ControllerCoreCommandType commandPossibility : getInverseKinematicsCommands())
      {
         if (commandType.equals(commandPossibility))
            return true;
      }

      return false;
   }

   /**
    * Polls whether the command {@code commandType} is an available command for the {@link WholeBodyInverseDynamicsSolver}.
    */
   public static boolean isInverseDynamicsCommand(ControllerCoreCommandType commandType)
   {
      for (ControllerCoreCommandType commandPossibility : getInverseDynamicsCommands())
      {
         if (commandType.equals(commandPossibility))
            return true;
      }

      return false;
   }

   /**
    * Polls whether the command {@code commandType} is an available command for the {@link WholeBodyVirtualModelControlSolver}.
    */
   public static boolean isVirtualModelControlCommand(ControllerCoreCommandType commandType)
   {
      for (ControllerCoreCommandType commandPossibility : getVirtualModelControlCommands())
      {
         if (commandType.equals(commandPossibility))
            return true;
      }

      return false;
   }

   /**
    * Polls whether the command {@code commandType} is an available command for the {@link WholeBodyFeedbackController}.
    */
   public static boolean isFeedbackControlCommand(ControllerCoreCommandType commandType)
   {
      for (ControllerCoreCommandType commandPossibility : getFeedbackControlCommands())
      {
         if (commandType.equals(commandPossibility))
            return true;
      }

      return false;
   }
}
