package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
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

   /**
    * In addition to clearing the list of commands declared in the super-type, it clears the internal
    * buffers marking the commands previously used as available.
    */
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

   /**
    * Unsupported operation.
    */
   @Override
   public void set(InverseDynamicsCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommand(InverseDynamicsCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommandList(InverseDynamicsCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public InverseDynamicsCommand<?> pollCommand()
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Gets an available {@link CenterOfPressureCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public CenterOfPressureCommand addCenterOfPressureCommand()
   {
      CenterOfPressureCommand command = centerOfPressureCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link ContactWrenchCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public ContactWrenchCommand addContactWrenchCommand()
   {
      ContactWrenchCommand command = contactWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link ExternalWrenchCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public ExternalWrenchCommand addExternalWrenchCommand()
   {
      ExternalWrenchCommand command = externalWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link InverseDynamicsOptimizationSettingsCommand} and registers it to this
    * list.
    * 
    * @return the available command ready to be set.
    */
   public InverseDynamicsOptimizationSettingsCommand addInverseDynamicsOptimizationSettingsCommand()
   {
      InverseDynamicsOptimizationSettingsCommand command = inverseDynamicsOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointAccelerationIntegrationCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointAccelerationIntegrationCommand addJointAccelerationIntegrationCommand()
   {
      JointAccelerationIntegrationCommand command = jointAccelerationIntegrationCommandBuffer.add();
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
    * Gets an available {@link JointspaceAccelerationCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointspaceAccelerationCommand addJointspaceAccelerationCommand()
   {
      JointspaceAccelerationCommand command = jointspaceAccelerationCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link MomentumRateCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public MomentumRateCommand addMomentumRateCommand()
   {
      MomentumRateCommand command = momentumRateCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link PlaneContactStateCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public PlaneContactStateCommand addPlaneContactStateCommand()
   {
      PlaneContactStateCommand command = planeContactStateCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link SpatialAccelerationCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public SpatialAccelerationCommand addSpatialAccelerationCommand()
   {
      SpatialAccelerationCommand command = spatialAccelerationCommandBuffer.add();
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
}
