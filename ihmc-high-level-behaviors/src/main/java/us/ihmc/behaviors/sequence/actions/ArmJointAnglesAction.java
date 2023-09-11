package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.Timer;

public class ArmJointAnglesAction extends ArmJointAnglesActionData implements BehaviorAction
{
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private int actionIndex;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();

   public ArmJointAnglesAction(DRCRobotModel robotModel, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      update();

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      double[] jointAngleArray;
      if (getPreset() == null)
      {
         jointAngleArray = new double[NUMBER_OF_JOINTS];
         for (int i = 0; i < NUMBER_OF_JOINTS; i++)
         {
            jointAngleArray[i] = getJointAngles()[i];
         }
      }
      else
      {
         jointAngleArray = robotModel.getPresetArmConfiguration(getSide(), getPreset());
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getSide(), getTrajectoryDuration(), jointAngleArray);
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      isExecuting = executionTimer.isRunning(getTrajectoryDuration());

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, this.executionStatusMessage);
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
}
