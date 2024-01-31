package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.tools.Timer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionExecutor extends ActionNodeExecutor<SakeHandCommandActionState, SakeHandCommandActionDefinition>
{
   /** TODO: Make this variable. */
   private static final double WAIT_TIME = 0.5;

   private final SakeHandCommandActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();

   public SakeHandCommandActionExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2ControllerHelper ros2ControllerHelper)
   {
      super(new SakeHandCommandActionState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      // FIXME: Needs major work
      if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.GOTO)
      {
         SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
         message.setRobotSide(getDefinition().getSide().toByte());
         message.setDesiredHandConfiguration((byte) SakeHandCommandOption.values[getDefinition().getHandConfigurationIndex()].getCommandNumber());
         message.setPostionRatio(getDefinition().getGoalPosition());
         message.setTorqueRatio(-1.0);

         ros2ControllerHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);

         message.setPostionRatio(-1.0);
         message.setTorqueRatio(getDefinition().getGoalTorque());

         ros2ControllerHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
      }
      else if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.OPEN)
      {
         ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                      HumanoidMessageTools.createHandDesiredConfigurationMessage(getDefinition().getSide(),
                                                                                                 HandConfiguration.OPEN));
      }
      else if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.CLOSE)
      {
         ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                      HumanoidMessageTools.createHandDesiredConfigurationMessage(getDefinition().getSide(),
                                                                                                 HandConfiguration.CLOSE));
      }

      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(WAIT_TIME));

      state.setNominalExecutionDuration(WAIT_TIME);
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }
}
