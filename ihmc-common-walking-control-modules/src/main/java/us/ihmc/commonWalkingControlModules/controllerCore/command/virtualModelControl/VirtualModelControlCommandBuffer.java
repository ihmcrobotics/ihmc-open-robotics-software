package us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;

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

   @Override
   public void set(VirtualModelControlCommandList other)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommand(VirtualModelControlCommand<?> command)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void addCommandList(VirtualModelControlCommandList commandList)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public VirtualModelControlCommand<?> pollCommand()
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

   public JointAccelerationIntegrationCommand addJointAccelerationIntegrationCommand()
   {
      JointAccelerationIntegrationCommand command = jointAccelerationIntegrationCommandBuffer.add();
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

   public JointLimitEnforcementCommand addJointLimitEnforcementCommand()
   {
      JointLimitEnforcementCommand command = jointLimitEnforcementCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public JointTorqueCommand addJointTorqueCommand()
   {
      JointTorqueCommand command = jointTorqueCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public VirtualForceCommand addVirtualForceCommand()
   {
      VirtualForceCommand command = virtualForceCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public VirtualTorqueCommand addVirtualTorqueCommand()
   {
      VirtualTorqueCommand command = virtualTorqueCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public VirtualWrenchCommand addVirtualWrenchCommand()
   {
      VirtualWrenchCommand command = virtualWrenchCommandBuffer.add();
      super.addCommand(command);
      return command;
   }

   public VirtualModelControlOptimizationSettingsCommand addVirtualModelControlOptimizationSettingsCommand()
   {
      VirtualModelControlOptimizationSettingsCommand command = virtualModelControlOptimizationSettingsCommandBuffer.add();
      super.addCommand(command);
      return command;
   }
}
