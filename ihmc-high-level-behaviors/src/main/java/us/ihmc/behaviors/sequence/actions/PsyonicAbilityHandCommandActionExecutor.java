package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.AbilityHandLegacyGripCommandMessage;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripSpeed;
import us.ihmc.abilityhand.AbilityHandLegacyGripCommand.LegacyGripType;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.AbilityHandAPI;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PsyonicAbilityHandCommandActionExecutor extends ActionNodeExecutor<PsyonicAbilityHandCommandActionState, PsyonicAbilityHandCommandActionDefinition>
{
   private final PsyonicAbilityHandCommandActionState state;
   private final PsyonicAbilityHandCommandActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;

   public PsyonicAbilityHandCommandActionExecutor(long id,
                                        CRDTInfo crdtInfo,
                                        WorkspaceResourceDirectory saveFileDirectory,
                                        ROS2ControllerHelper ros2ControllerHelper,
                                        ROS2SyncedRobotModel syncedRobot)
   {
      super(new PsyonicAbilityHandCommandActionState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void update()
   {
      super.update();

      state.setCanExecute(true);
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      sendLegacyGrip(definition.getLegacyGripType(), definition.getLegacyGripSpeed(), definition.getSide());
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false);
   }

   private void sendLegacyGrip(LegacyGripType gripType, LegacyGripSpeed gripSpeed, RobotSide hand)
   {
      AbilityHandLegacyGripCommandMessage message = new AbilityHandLegacyGripCommandMessage();
      message.setLegacyGripType(gripType.getAsciiGripIndex());
      message.setLegacyGripSpeed(gripSpeed.name());

      ros2ControllerHelper.publish(AbilityHandAPI.getAbilityHandLegacyGripCommandTopic(syncedRobot.getRobotModel().getSimpleRobotName(), hand), message);
   }
}
