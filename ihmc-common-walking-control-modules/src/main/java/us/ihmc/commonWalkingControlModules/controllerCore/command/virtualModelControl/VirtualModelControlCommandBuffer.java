package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.CrossRobotCommandResolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
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
public class VirtualModelControlCommandBuffer extends VirtualModelControlCommandList
{
   private final RecyclingArrayList<CenterOfPressureCommand> centerOfPressureCommandBuffer = new RecyclingArrayList<>(CenterOfPressureCommand.class);
   private final RecyclingArrayList<ContactWrenchCommand> contactWrenchCommandBuffer = new RecyclingArrayList<>(ContactWrenchCommand.class);
   private final RecyclingArrayList<PlaneContactStateCommand> planeContactStateCommandBuffer = new RecyclingArrayList<>(PlaneContactStateCommand.class);
   private final RecyclingArrayList<MomentumRateCommand> momentumRateCommandBuffer = new RecyclingArrayList<>(MomentumRateCommand.class);
   private final RecyclingArrayList<ExternalWrenchCommand> externalWrenchCommandBuffer = new RecyclingArrayList<>(ExternalWrenchCommand.class);
   private final RecyclingArrayList<JointAccelerationIntegrationCommand> jointAccelerationIntegrationCommandBuffer = new RecyclingArrayList<>(JointAccelerationIntegrationCommand.class);
   private final RecyclingArrayList<JointLimitEnforcementCommand> jointLimitEnforcementCommandBuffer = new RecyclingArrayList<>(JointLimitEnforcementCommand.class);
   private final RecyclingArrayList<JointTorqueCommand> jointTorqueCommandBuffer = new RecyclingArrayList<>(JointTorqueCommand.class);
   private final RecyclingArrayList<VirtualForceCommand> virtualForceCommandBuffer = new RecyclingArrayList<>(VirtualForceCommand.class);
   private final RecyclingArrayList<VirtualTorqueCommand> virtualTorqueCommandBuffer = new RecyclingArrayList<>(VirtualTorqueCommand.class);
   private final RecyclingArrayList<VirtualWrenchCommand> virtualWrenchCommandBuffer = new RecyclingArrayList<>(VirtualWrenchCommand.class);
   private final RecyclingArrayList<VirtualModelControlOptimizationSettingsCommand> virtualModelControlOptimizationSettingsCommandBuffer = new RecyclingArrayList<>(VirtualModelControlOptimizationSettingsCommand.class);

   public VirtualModelControlCommandBuffer()
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
      planeContactStateCommandBuffer.clear();
      momentumRateCommandBuffer.clear();
      externalWrenchCommandBuffer.clear();
      jointAccelerationIntegrationCommandBuffer.clear();
      jointLimitEnforcementCommandBuffer.clear();
      jointTorqueCommandBuffer.clear();
      virtualForceCommandBuffer.clear();
      virtualTorqueCommandBuffer.clear();
      virtualWrenchCommandBuffer.clear();
      virtualModelControlOptimizationSettingsCommandBuffer.clear();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void set(VirtualModelControlCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommand(VirtualModelControlCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public void addCommandList(VirtualModelControlCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   /**
    * Unsupported operation.
    */
   @Override
   public VirtualModelControlCommand<?> pollCommand()
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
    * Gets an available {@link JointLimitEnforcementCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointLimitEnforcementCommand addJointLimitEnforcementCommand()
   {
      JointLimitEnforcementCommand command = jointLimitEnforcementCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link JointTorqueCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public JointTorqueCommand addJointTorqueCommand()
   {
      JointTorqueCommand command = jointTorqueCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link VirtualForceCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public VirtualForceCommand addVirtualForceCommand()
   {
      VirtualForceCommand command = virtualForceCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link VirtualTorqueCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public VirtualTorqueCommand addVirtualTorqueCommand()
   {
      VirtualTorqueCommand command = virtualTorqueCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link VirtualWrenchCommand} and registers it to this list.
    * 
    * @return the available command ready to be set.
    */
   public VirtualWrenchCommand addVirtualWrenchCommand()
   {
      VirtualWrenchCommand command = virtualWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   /**
    * Gets an available {@link VirtualModelControlOptimizationSettingsCommand} and registers it to this
    * list.
    * 
    * @return the available command ready to be set.
    */
   public VirtualModelControlOptimizationSettingsCommand addVirtualModelControlOptimizationSettingsCommand()
   {
      VirtualModelControlOptimizationSettingsCommand command = virtualModelControlOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
}
