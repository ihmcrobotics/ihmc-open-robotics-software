package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionExecutor;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.Timer;

public class ArmJointAnglesActionExecutor implements BehaviorActionExecutor
{
   private final ArmJointAnglesActionState state = new ArmJointAnglesActionState();
   private final ArmJointAnglesActionDefinition definition = state.getDefinition();
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private int actionIndex;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public ArmJointAnglesActionExecutor(DRCRobotModel robotModel, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex, boolean concurrentActionIsNextForExecution)
   {
      this.actionIndex = actionIndex;
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
      isExecuting = executionTimer.isRunning(definition.getTrajectoryDuration());

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(definition.getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public ActionExecutionStatusMessage getExecutionStatusMessage()
   {
      return executionStatusMessage;
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
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
