package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandCommandOption;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.JointspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class SakeHandCommandActionExecutor extends ActionNodeExecutor<SakeHandCommandActionState, SakeHandCommandActionDefinition>
{
   /** TODO: Make this variable. */
   private static final double WAIT_TIME = 2.5;
   public static final double ANGLE_TOLERANCE = Math.toRadians(5.0);

   private final SakeHandCommandActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final JointspaceTrajectoryTrackingErrorCalculator trackingCalculator = new JointspaceTrajectoryTrackingErrorCalculator();
   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();

   public SakeHandCommandActionExecutor(long id,
                                        CRDTInfo crdtInfo,
                                        WorkspaceResourceDirectory saveFileDirectory,
                                        ROS2ControllerHelper ros2ControllerHelper,
                                        ROS2SyncedRobotModel syncedRobot)
   {
      super(new SakeHandCommandActionState(id, crdtInfo, saveFileDirectory));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      for (RobotSide side : RobotSide.values)
      {
         x1KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(0));
         x2KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(1));
      }
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));
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

      trackingCalculator.reset();

      double inputOpenAmountForSideZeroToHalf = getDefinition().getGoalPosition();
      double goalJointAngle = inputOpenAmountForSideZeroToHalf * Math.toRadians(SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS);
      double goalOpenAngle = goalJointAngle * 2.0;

      LogTools.info("Commanding hand to position %.2f%s".formatted(Math.toDegrees(goalOpenAngle), EuclidCoreMissingTools.DEGREE_SYMBOL));

      state.getDesiredJointTrajectories().clear(2);
      state.getDesiredJointTrajectories().addTrajectoryPoint(0, x1KnuckleJoints.get(getDefinition().getSide()).getQ(), 0.0);
      state.getDesiredJointTrajectories().addTrajectoryPoint(0, goalJointAngle, WAIT_TIME);
      state.getDesiredJointTrajectories().addTrajectoryPoint(1, x2KnuckleJoints.get(getDefinition().getSide()).getQ(), 0.0);
      state.getDesiredJointTrajectories().addTrajectoryPoint(1, goalJointAngle, WAIT_TIME);
      state.setNominalExecutionDuration(WAIT_TIME);
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());
      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         LogTools.error("Task execution timed out.");
         return;
      }

      state.getCurrentJointAngles().getValue()[0] = x1KnuckleJoints.get(getDefinition().getSide()).getQ();
      state.getCurrentJointAngles().getValue()[1] = x2KnuckleJoints.get(getDefinition().getSide()).getQ();

//      trackingCalculator.addJointData(syncedRobot.getFullRobotModel().get);
//      trackingCalculator.applyTolerance(ANGLE_TOLERANCE);

//      boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
//      meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();

      boolean meetsDesiredCompletionCriteria = trackingCalculator.getTimeIsUp();

      if (meetsDesiredCompletionCriteria)
      {
         state.setIsExecuting(false);
      }
   }
}
