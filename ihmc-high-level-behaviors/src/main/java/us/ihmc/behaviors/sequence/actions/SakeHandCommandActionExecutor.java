package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.sequence.JointspaceTrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.crdt.CRDTInfo;
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
   private static final double NOMINAL_TRAJECORY_DURATION = 1.0;

   private final SakeHandCommandActionState state;
   private final SakeHandCommandActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final JointspaceTrajectoryTrackingErrorCalculator trackingCalculator = new JointspaceTrajectoryTrackingErrorCalculator();
   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();
   private final SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();

   public SakeHandCommandActionExecutor(long id,
                                        CRDTInfo crdtInfo,
                                        WorkspaceResourceDirectory saveFileDirectory,
                                        ROS2ControllerHelper ros2ControllerHelper,
                                        ROS2SyncedRobotModel syncedRobot)
   {
      super(new SakeHandCommandActionState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasSakeGripperJoints(side))
         {
            x1KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(0));
            x2KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(1));
         }
      }
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      boolean canExecute = x1KnuckleJoints.get(definition.getSide()) != null;
      canExecute &= x2KnuckleJoints.get(definition.getSide()) != null;
      canExecute &= syncedRobot.getSakeHandStatus().get(definition.getSide()).getIsCalibrated();
      canExecute &= !syncedRobot.getSakeHandStatus().get(definition.getSide()).getNeedsReset();
      state.setCanExecute(canExecute);
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      trackingCalculator.reset();

      trackingCalculator.resetErrorMeasurement();
      double goalKnuckleJointAngle = SakeHandParameters.handOpenAngleToKnuckleJointAngle(definition.getHandOpenAngle());
      trackingCalculator.addJointData(x1KnuckleJoints.get(definition.getSide()).getQ(), goalKnuckleJointAngle);
      trackingCalculator.addJointData(x2KnuckleJoints.get(definition.getSide()).getQ(), goalKnuckleJointAngle);
      trackingCalculator.applyTolerance(definition.getInitialSatisfactionHandAngleTolerance());

      LogTools.info("x1: %.2f%s  x2: %.2f%s  Goal open angle: %.2f%s Error: %.2f%s"
                          .formatted(Math.toDegrees(x1KnuckleJoints.get(definition.getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(x2KnuckleJoints.get(definition.getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(definition.getHandOpenAngle()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(trackingCalculator.getTotalAbsolutePositionError()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL));

      if (trackingCalculator.isWithinPositionTolerance())
      {
         LogTools.info("Gripper is already at the desired position. Proceeding to next action. (Error: %.2f)"
                             .formatted(trackingCalculator.getTotalAbsolutePositionError()));
         state.setNominalExecutionDuration(0.0);
         state.setPositionDistanceToGoalTolerance(definition.getInitialSatisfactionHandAngleTolerance());
      }
      else
      {
         sakeHandDesiredCommandMessage.setRobotSide(definition.getSide().toByte());
//         SakeHandParameters.resetDesiredCommandMessage(sakeHandDesiredCommandMessage); FIXME: make this the new ethersnacks message
         sakeHandDesiredCommandMessage.setNormalizedGripperDesiredPosition(SakeHandParameters.normalizeHandOpenAngle(definition.getHandOpenAngle()));
         sakeHandDesiredCommandMessage.setNormalizedGripperTorqueLimit(
                                                       SakeHandParameters.normalizeFingertipGripForceLimit(definition.getFingertipGripForceLimit()));
         ros2ControllerHelper.publish(robotName -> ROS2Tools.getHandSakeCommandTopic(robotName, definition.getSide()), sakeHandDesiredCommandMessage);

         LogTools.info("Commanding hand to open angle %.2f%s with torque limit %.2f N".formatted(Math.toDegrees(definition.getHandOpenAngle()),
                                                                                                 EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                                                                 definition.getFingertipGripForceLimit()));

         state.getCommandedJointTrajectories().clear(2);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, x1KnuckleJoints.get(definition.getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, goalKnuckleJointAngle, NOMINAL_TRAJECORY_DURATION);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, x2KnuckleJoints.get(definition.getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, goalKnuckleJointAngle, NOMINAL_TRAJECORY_DURATION);
         state.setNominalExecutionDuration(NOMINAL_TRAJECORY_DURATION);
         state.setPositionDistanceToGoalTolerance(definition.getCompletionHandAngleTolerance());
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());

      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());
      state.getCurrentJointAngles().getValue()[0] = x1KnuckleJoints.get(definition.getSide()).getQ();
      state.getCurrentJointAngles().getValue()[1] = x2KnuckleJoints.get(definition.getSide()).getQ();

      if (trackingCalculator.getHitTimeLimit())
      {
         state.setFailed(true);
         state.setIsExecuting(false);
         LogTools.error("Task execution timed out.");
      }
      else if (!state.getCommandedJointTrajectories().isEmpty())
      {
         trackingCalculator.resetErrorMeasurement();
         trackingCalculator.addJointData(x1KnuckleJoints.get(definition.getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(0).getPosition());
         trackingCalculator.addJointData(x2KnuckleJoints.get(definition.getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(1).getPosition());
         trackingCalculator.applyTolerance(state.getPositionDistanceToGoalTolerance());

         boolean meetsDesiredCompletionCriteria = trackingCalculator.isWithinPositionTolerance();
         meetsDesiredCompletionCriteria &= trackingCalculator.getTimeIsUp();
         state.setIsExecuting(!meetsDesiredCompletionCriteria);
      }
      else // Hand already in desired configuration
      {
         state.setIsExecuting(false);
      }
   }
}
