package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commons.lists.RecyclingArrayList;

/**
 * This class is not for general user, it is used for performing cross-robot command conversion in a
 * garbage free manner.
 * <p>
 * This class should only be used with {@link CrossRobotCommandResolver} and
 * {@link ControllerCoreCommandBuffer} to resolve a {@link ControllerCoreCommand}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class InverseKinematicsCommandBuffer extends InverseKinematicsCommandList
{
   private final RecyclingArrayList<InverseKinematicsOptimizationSettingsCommand> inverseKinematicsOptimizationSettingsCommandBuffer = new RecyclingArrayList<>(InverseKinematicsOptimizationSettingsCommand.class);
   private final RecyclingArrayList<JointLimitReductionCommand> jointLimitReductionCommandBuffer = new RecyclingArrayList<>(JointLimitReductionCommand.class);
   private final RecyclingArrayList<JointLimitEnforcementMethodCommand> jointLimitEnforcementMethodCommandBuffer = new RecyclingArrayList<>(JointLimitEnforcementMethodCommand.class);
   private final RecyclingArrayList<JointspaceVelocityCommand> jointspaceVelocityCommandBuffer = new RecyclingArrayList<>(JointspaceVelocityCommand.class);
   private final RecyclingArrayList<MomentumCommand> momentumCommandBuffer = new RecyclingArrayList<>(MomentumCommand.class);
   private final RecyclingArrayList<LinearMomentumConvexConstraint2DCommand> linearMomentumConvexConstraint2DCommandBuffer = new RecyclingArrayList<>(LinearMomentumConvexConstraint2DCommand.class);
   private final RecyclingArrayList<PrivilegedConfigurationCommand> privilegedConfigurationCommandBuffer = new RecyclingArrayList<>(PrivilegedConfigurationCommand.class);
   private final RecyclingArrayList<PrivilegedJointSpaceCommand> privilegedJointSpaceCommandBuffer = new RecyclingArrayList<>(PrivilegedJointSpaceCommand.class);
   private final RecyclingArrayList<SpatialVelocityCommand> spatialVelocityCommandBuffer = new RecyclingArrayList<>(SpatialVelocityCommand.class);

   public InverseKinematicsCommandBuffer()
   {
   }

   /**
    * In addition to clearing the list of commands declared in the super-type, it clears the internal
    * buffers marking the commands previously used as available.
    */
   @Override
   public void clear()
   {
      super.clear();
      inverseKinematicsOptimizationSettingsCommandBuffer.clear();
      jointLimitReductionCommandBuffer.clear();
      jointLimitEnforcementMethodCommandBuffer.clear();
      jointspaceVelocityCommandBuffer.clear();
      momentumCommandBuffer.clear();
      linearMomentumConvexConstraint2DCommandBuffer.clear();
      privilegedConfigurationCommandBuffer.clear();
      privilegedJointSpaceCommandBuffer.clear();
      spatialVelocityCommandBuffer.clear();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void set(InverseKinematicsCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommand(InverseKinematicsCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommandList(InverseKinematicsCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public InverseKinematicsCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Gets an available {@link InverseKinematicsOptimizationSettingsCommand} and registers it to this
    * list.
    * 
    * @return the available command ready to be set.
    */
   public InverseKinematicsOptimizationSettingsCommand addInverseKinematicsOptimizationSettingsCommand()
   {
      InverseKinematicsOptimizationSettingsCommand command = inverseKinematicsOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointLimitReductionCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointLimitReductionCommand addJointLimitReductionCommand()
   {
      JointLimitReductionCommand command = jointLimitReductionCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointLimitEnforcementMethodCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointLimitEnforcementMethodCommand addJointLimitEnforcementMethodCommand()
   {
      JointLimitEnforcementMethodCommand command = jointLimitEnforcementMethodCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointspaceVelocityCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointspaceVelocityCommand addJointspaceVelocityCommand()
   {
      JointspaceVelocityCommand command = jointspaceVelocityCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link MomentumCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public MomentumCommand addMomentumCommand()
   {
      MomentumCommand command = momentumCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
   
   /**
    * Gets an available {@link LinearMomentumConvexConstraint2DCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public LinearMomentumConvexConstraint2DCommand addLinearMomentumConvexConstraint2DCommand()
   {
      LinearMomentumConvexConstraint2DCommand command = linearMomentumConvexConstraint2DCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link PrivilegedConfigurationCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public PrivilegedConfigurationCommand addPrivilegedConfigurationCommand()
   {
      PrivilegedConfigurationCommand command = privilegedConfigurationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link PrivilegedJointSpaceCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public PrivilegedJointSpaceCommand addPrivilegedJointSpaceCommand()
   {
      PrivilegedJointSpaceCommand command = privilegedJointSpaceCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link SpatialVelocityCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public SpatialVelocityCommand addSpatialVelocityCommand()
   {
      SpatialVelocityCommand command = spatialVelocityCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

}
