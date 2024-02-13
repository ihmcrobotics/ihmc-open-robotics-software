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
   /**
    * This is the typically how long the basic OPEN and CLOSE commands take on the real robot.
    * TODO: Make this derivative of the command by adding and supporting
    *   desired finger velocities.
    */
   private static final double NOMINAL_TRAJECORY_DURATION = 2.5;
   public static final double ANGLE_TOLERANCE = Math.toRadians(40.0); // We want to allow a bunch of compliance
   /** If it's already within 5 degrees, we will just mark is as completed. */
   public static final double INITIAL_SATISFACTION_TOLERANCE = Math.toRadians(5.0);

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

      double inputOpenAmountForSideZeroToHalf = getDefinition().getHandOpenAngle();
      double goalJointAngle = inputOpenAmountForSideZeroToHalf * Math.toRadians(SakeHandParameters.MAX_ANGLE_BETWEEN_FINGERS);
      double goalOpenAngle = goalJointAngle * 2.0;

      trackingCalculator.reset();

      trackingCalculator.resetErrorMeasurement();
      trackingCalculator.addJointData(x1KnuckleJoints.get(getDefinition().getSide()).getQ(), goalJointAngle);
      trackingCalculator.addJointData(x2KnuckleJoints.get(getDefinition().getSide()).getQ(), goalJointAngle);
      trackingCalculator.applyTolerance(INITIAL_SATISFACTION_TOLERANCE);

      LogTools.info("x1: %.2f%s  x2: %.2f%s  Goal position: %.2f%s Option: %.2f%s  Error: %.2f%s"
                          .formatted(Math.toDegrees(x1KnuckleJoints.get(getDefinition().getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(x2KnuckleJoints.get(getDefinition().getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(goalJointAngle),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(getDefinition().getSakeCommandOption().getNormalizedHandOpenAngle()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(trackingCalculator.getTotalAbsolutePositionError()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL));

      if (trackingCalculator.isWithinPositionTolerance())
      {
         LogTools.info("Gripper is already at the desired position. Proceeding to next action. (Error: %.2f)"
                             .formatted(trackingCalculator.getTotalAbsolutePositionError()));
         state.setNominalExecutionDuration(0.0);
         state.setPositionDistanceToGoalTolerance(INITIAL_SATISFACTION_TOLERANCE);
      }
      else
      {
         if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.GOTO)
         {
            // FIXME: Needs major work
            SakeHandDesiredCommandMessage message = new SakeHandDesiredCommandMessage();
            message.setRobotSide(getDefinition().getSide().toByte());
            message.setDesiredHandConfiguration((byte) SakeHandCommandOption.values[getDefinition().getHandConfigurationIndex()].getCommandNumber());
            message.setPostionRatio(getDefinition().getHandOpenAngle());
            message.setTorqueRatio(-1.0);

            ros2ControllerHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);

            message.setPostionRatio(-1.0);
            message.setTorqueRatio(getDefinition().getMaxTorque());

            LogTools.info("Commanding hand to GOTO position %.2f%s".formatted(Math.toDegrees(goalOpenAngle),
                                                                         EuclidCoreMissingTools.DEGREE_SYMBOL));
            ros2ControllerHelper.publish(ROS2Tools::getHandSakeCommandTopic, message);
         }
         else if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.OPEN)
         {
            LogTools.info("Commanding hand to OPEN position %.2f%s".formatted(Math.toDegrees(goalOpenAngle),
                                                                              EuclidCoreMissingTools.DEGREE_SYMBOL));
            ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                         HumanoidMessageTools.createHandDesiredConfigurationMessage(getDefinition().getSide(),
                                                                                                    HandConfiguration.OPEN));
         }
         else if (getDefinition().getSakeCommandOption() == SakeHandCommandOption.CLOSE)
         {
            LogTools.info("Commanding hand to CLOSE position %.2f%s".formatted(Math.toDegrees(goalOpenAngle),
                                                                               EuclidCoreMissingTools.DEGREE_SYMBOL));
            ros2ControllerHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                         HumanoidMessageTools.createHandDesiredConfigurationMessage(getDefinition().getSide(),
                                                                                                    HandConfiguration.CLOSE));
         }

         state.getCommandedJointTrajectories().clear(2);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, x1KnuckleJoints.get(getDefinition().getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, goalJointAngle, NOMINAL_TRAJECORY_DURATION);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, x2KnuckleJoints.get(getDefinition().getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, goalJointAngle, NOMINAL_TRAJECORY_DURATION);
         state.setNominalExecutionDuration(NOMINAL_TRAJECORY_DURATION);
         state.setPositionDistanceToGoalTolerance(ANGLE_TOLERANCE);
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());

      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());
      state.getCurrentJointAngles().getValue()[0] = x1KnuckleJoints.get(getDefinition().getSide()).getQ();
      state.getCurrentJointAngles().getValue()[1] = x2KnuckleJoints.get(getDefinition().getSide()).getQ();

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setFailed(true);
         state.setIsExecuting(false);
         LogTools.error("Task execution timed out.");
      }
      else if (!state.getCommandedJointTrajectories().isEmpty())
      {
         trackingCalculator.resetErrorMeasurement();
         trackingCalculator.addJointData(x1KnuckleJoints.get(getDefinition().getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(0).getPosition());
         trackingCalculator.addJointData(x2KnuckleJoints.get(getDefinition().getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(1).getPosition());
         trackingCalculator.applyTolerance(state.getPositionDistanceToGoalTolerance());

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.setIsExecuting(meetsDesiredCompletionCriteria);
      }
      else // Hand already in desired configuration
      {
         state.setIsExecuting(false);
      }
   }
}
