package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.Timer;

public class ArmJointAnglesActionExecutor extends BehaviorActionExecutor
{
   private final ArmJointAnglesActionDefinition definition = new ArmJointAnglesActionDefinition();
   private final ArmJointAnglesActionState state;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public ArmJointAnglesActionExecutor(long id,
                                       BehaviorActionSequence sequence,
                                       DRCRobotModel robotModel,
                                       ROS2ControllerHelper ros2ControllerHelper)
   {
      super(sequence);

      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;

      state = new ArmJointAnglesActionState(id, definition);
   }

   @Override
   public void update()
   {
      super.update();
   }

   @Override
   public void triggerActionExecution()
   {
      double[] jointAngleArray;
      if (state.getDefinition().getPreset() == null)
      {
         jointAngleArray = new double[ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS];
         for (int i = 0; i < ArmJointAnglesActionDefinition.NUMBER_OF_JOINTS; i++)
         {
            jointAngleArray[i] = definition.getJointAngles()[i];
         }
      }
      else
      {
         jointAngleArray = robotModel.getPresetArmConfiguration(definition.getSide(), definition.getPreset());
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(definition.getSide(),
                                                                                                  definition.getTrajectoryDuration(),
                                                                                                  jointAngleArray);
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(executionTimer.isRunning(definition.getTrajectoryDuration()));

      executionStatusMessage.setActionIndex(state.getActionIndex());
      executionStatusMessage.setNominalExecutionDuration(definition.getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public ArmJointAnglesActionState getState()
   {
      return state;
   }

   @Override
   public ArmJointAnglesActionDefinition getDefinition()
   {
      return definition;
   }
}
