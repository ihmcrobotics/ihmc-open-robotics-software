package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.tools.Timer;

public class ArmJointAnglesActionExecutor extends ActionNodeExecutor<ArmJointAnglesActionState, ArmJointAnglesActionDefinition>
{
   private final ArmJointAnglesActionState state;
   private final DRCRobotModel robotModel;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final Timer executionTimer = new Timer();

   public ArmJointAnglesActionExecutor(long id, CRDTInfo crdtInfo, DRCRobotModel robotModel, ROS2ControllerHelper ros2ControllerHelper)
   {
      this.robotModel = robotModel;
      this.ros2ControllerHelper = ros2ControllerHelper;

      state = new ArmJointAnglesActionState(id, crdtInfo);
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
            jointAngleArray[i] = getDefinition().getJointAngles()[i];
         }
      }
      else
      {
         jointAngleArray = robotModel.getPresetArmConfiguration(getDefinition().getSide(), getDefinition().getPreset());
      }
      ArmTrajectoryMessage armTrajectoryMessage = HumanoidMessageTools.createArmTrajectoryMessage(getDefinition().getSide(),
                                                                                                  getDefinition().getTrajectoryDuration(),
                                                                                                  jointAngleArray);
      ros2ControllerHelper.publishToController(armTrajectoryMessage);
      executionTimer.reset();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setActionIndex(state.getActionIndex());
      state.setIsExecuting(executionTimer.isRunning(getDefinition().getTrajectoryDuration()));
      state.setNominalExecutionDuration(getDefinition().getTrajectoryDuration());
      state.setElapsedExecutionTime(executionTimer.getElapsedTime());
   }

   @Override
   public ArmJointAnglesActionState getState()
   {
      return state;
   }
}
