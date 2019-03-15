package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commons.lists.RecyclingArrayList;

public class InverseDynamicsCommandBuffer extends InverseDynamicsCommandList
{
   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommandBuffer = new RecyclingArrayList<>(CenterOfPressureCommand.class);
   private final RecyclingArrayList<ContactWrenchCommand> contactWrenchCommandBuffer = new RecyclingArrayList<>(ContactWrenchCommand.class);
   private final RecyclingArrayList<ExternalWrenchCommand> externalWrenchCommandBuffer = new RecyclingArrayList<>(ExternalWrenchCommand.class);
   private final RecyclingArrayList<InverseDynamicsOptimizationSettingsCommand> inverseDynamicsOptimizationSettingsCommandBuffer = new RecyclingArrayList<>(InverseDynamicsOptimizationSettingsCommand.class);
   private final RecyclingArrayList<JointAccelerationIntegrationCommand> jointAccelerationIntegrationCommandBuffer = new RecyclingArrayList<>(JointAccelerationIntegrationCommand.class);
   private final RecyclingArrayList<JointLimitEnforcementMethodCommand> jointLimitEnforcementMethodCommandBuffer = new RecyclingArrayList<>(JointLimitEnforcementMethodCommand.class);
   private final RecyclingArrayList<JointspaceAccelerationCommand> jointspaceAccelerationCommandBuffer = new RecyclingArrayList<>(JointspaceAccelerationCommand.class);
   private final RecyclingArrayList<MomentumRateCommand> momentumRateCommandBuffer = new RecyclingArrayList<>(MomentumRateCommand.class);
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandBuffer = new RecyclingArrayList<>(PlaneContactStateCommand.class);
   private final RecyclingArrayList<SpatialAccelerationCommand> spatialAccelerationCommandBuffer = new RecyclingArrayList<>(SpatialAccelerationCommand.class);
   private final RecyclingArrayList<JointLimitReductionCommand> jointLimitReductionCommandBuffer = new RecyclingArrayList<>(JointLimitReductionCommand.class);
   private final RecyclingArrayList<PrivilegedJointSpaceCommand> privilegedJointSpaceCommandBuffer = new RecyclingArrayList<>(PrivilegedJointSpaceCommand.class);
   private final RecyclingArrayList<PrivilegedConfigurationCommand> privilegedConfigurationCommandBuffer = new RecyclingArrayList<>(PrivilegedConfigurationCommand.class);

   public InverseDynamicsCommandBuffer()
   {
   }

   @Override
   public void clear()
   {
      super.clear();
      centerOfPressureCommandBuffer.clear();
      contactWrenchCommandBuffer.clear();
      externalWrenchCommandBuffer.clear();
      inverseDynamicsOptimizationSettingsCommandBuffer.clear();
      jointAccelerationIntegrationCommandBuffer.clear();
      jointLimitEnforcementMethodCommandBuffer.clear();
      jointspaceAccelerationCommandBuffer.clear();
      momentumRateCommandBuffer.clear();
      planeContactStateCommandBuffer.clear();
      spatialAccelerationCommandBuffer.clear();
      jointLimitReductionCommandBuffer.clear();
      privilegedJointSpaceCommandBuffer.clear();
      privilegedConfigurationCommandBuffer.clear();
   }

   @Override
   public void set(InverseDynamicsCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommand(InverseDynamicsCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommandList(InverseDynamicsCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public InverseDynamicsCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   public CenterOfPressureCommand addCenterOfPressureCommand()
   {
      CenterOfPressureCommand command = centerOfPressureCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public ContactWrenchCommand addContactWrenchCommand()
   {
      ContactWrenchCommand command = contactWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public ExternalWrenchCommand addExternalWrenchCommand()
   {
      ExternalWrenchCommand command = externalWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public InverseDynamicsOptimizationSettingsCommand addInverseDynamicsOptimizationSettingsCommand()
   {
      InverseDynamicsOptimizationSettingsCommand command = inverseDynamicsOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointAccelerationIntegrationCommand addJointAccelerationIntegrationCommand()
   {
      JointAccelerationIntegrationCommand command = jointAccelerationIntegrationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointLimitEnforcementMethodCommand addJointLimitEnforcementMethodCommand()
   {
      JointLimitEnforcementMethodCommand command = jointLimitEnforcementMethodCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointspaceAccelerationCommand addJointspaceAccelerationCommand()
   {
      JointspaceAccelerationCommand command = jointspaceAccelerationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public MomentumRateCommand addMomentumRateCommand()
   {
      MomentumRateCommand command = momentumRateCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PlaneContactStateCommand addPlaneContactStateCommand()
   {
      PlaneContactStateCommand command = planeContactStateCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public SpatialAccelerationCommand addSpatialAccelerationCommand()
   {
      SpatialAccelerationCommand command = spatialAccelerationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointLimitReductionCommand addJointLimitReductionCommand()
   {
      JointLimitReductionCommand command = jointLimitReductionCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PrivilegedJointSpaceCommand addPrivilegedJointSpaceCommand()
   {
      PrivilegedJointSpaceCommand command = privilegedJointSpaceCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public PrivilegedConfigurationCommand addPrivilegedConfigurationCommand()
   {
      PrivilegedConfigurationCommand command = privilegedConfigurationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
}
