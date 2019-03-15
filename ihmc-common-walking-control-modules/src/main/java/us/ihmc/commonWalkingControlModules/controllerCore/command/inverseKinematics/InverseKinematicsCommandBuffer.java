package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commons.lists.RecyclingArrayList;

public class InverseKinematicsCommandBuffer extends InverseKinematicsCommandList
{
   private final RecyclingArrayList<InverseKinematicsOptimizationSettingsCommand> inverseKinematicsOptimizationSettingsCommandBuffer = new RecyclingArrayList<>(InverseKinematicsOptimizationSettingsCommand.class);
   private final RecyclingArrayList<JointLimitReductionCommand> jointLimitReductionCommandBuffer = new RecyclingArrayList<>(JointLimitReductionCommand.class);
   private final RecyclingArrayList<JointLimitEnforcementMethodCommand> jointLimitEnforcementMethodCommandBuffer = new RecyclingArrayList<>(JointLimitEnforcementMethodCommand.class);
   private final RecyclingArrayList<JointspaceVelocityCommand> jointspaceVelocityCommandBuffer = new RecyclingArrayList<>(JointspaceVelocityCommand.class);
   private final RecyclingArrayList<MomentumCommand> momentumCommandBuffer = new RecyclingArrayList<>(MomentumCommand.class);
   private final RecyclingArrayList<PrivilegedConfigurationCommand> privilegedConfigurationCommandBuffer = new RecyclingArrayList<>(PrivilegedConfigurationCommand.class);
   private final RecyclingArrayList<PrivilegedJointSpaceCommand> privilegedJointSpaceCommandBuffer = new RecyclingArrayList<>(PrivilegedJointSpaceCommand.class);
   private final RecyclingArrayList<SpatialVelocityCommand> spatialVelocityCommandBuffer = new RecyclingArrayList<>(SpatialVelocityCommand.class);

   public InverseKinematicsCommandBuffer()
   {
   }

   @Override
   public void clear()
   {
      super.clear();
      inverseKinematicsOptimizationSettingsCommandBuffer.clear();
      jointLimitReductionCommandBuffer.clear();
      jointLimitEnforcementMethodCommandBuffer.clear();
      jointspaceVelocityCommandBuffer.clear();
      momentumCommandBuffer.clear();
      privilegedConfigurationCommandBuffer.clear();
      privilegedJointSpaceCommandBuffer.clear();
      spatialVelocityCommandBuffer.clear();
   }

   @Override
   public void set(InverseKinematicsCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommand(InverseKinematicsCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommandList(InverseKinematicsCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public InverseKinematicsCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   public InverseKinematicsOptimizationSettingsCommand addInverseKinematicsOptimizationSettingsCommand()
   {
      InverseKinematicsOptimizationSettingsCommand command = inverseKinematicsOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointLimitReductionCommand addJointLimitReductionCommand()
   {
      JointLimitReductionCommand command = jointLimitReductionCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointLimitEnforcementMethodCommand addJointLimitEnforcementMethodCommand()
   {
      JointLimitEnforcementMethodCommand command = jointLimitEnforcementMethodCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointspaceVelocityCommand addJointspaceVelocityCommand()
   {
      JointspaceVelocityCommand command = jointspaceVelocityCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public MomentumCommand addMomentumCommand()
   {
      MomentumCommand command = momentumCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PrivilegedConfigurationCommand addPrivilegedConfigurationCommand()
   {
      PrivilegedConfigurationCommand command = privilegedConfigurationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PrivilegedJointSpaceCommand addPrivilegedJointSpaceCommand()
   {
      PrivilegedJointSpaceCommand command = privilegedJointSpaceCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public SpatialVelocityCommand addSpatialVelocityCommand()
   {
      SpatialVelocityCommand command = spatialVelocityCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

}
