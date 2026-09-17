package us.ihmc.behaviors.behaviorTree.action.actions;

import controller_msgs.SakeHandDesiredCommandMessage;
import us.ihmc.avatar.sakeGripper.ROS2SakeHandStatus;
import us.ihmc.avatar.sakeGripper.SakeHandParameters;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.TrajectoryTrackingErrorCalculator;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.SakeHandAPI;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EZGripperActionExecutor extends ActionNodeExecutor<EZGripperActionState, EZGripperActionDefinition>
{
   /**
    * This is the typically how long the basic OPEN and CLOSE commands take on the real robot.
    * TODO: Make this derivative of the command by adding and supporting
    *   desired finger velocities.
    */
   private static final double NOMINAL_TRAJECTORY_DURATION = 1.0;

   private final SideDependentList<ROS2SakeHandStatus> sakeHandStatus = new SideDependentList<>();
   private final TrajectoryTrackingErrorCalculator trackingCalculator = new TrajectoryTrackingErrorCalculator();
   private final SideDependentList<RevoluteJoint> x1KnuckleJoints = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint> x2KnuckleJoints = new SideDependentList<>();
   private final SakeHandDesiredCommandMessage sakeHandDesiredCommandMessage = new SakeHandDesiredCommandMessage();

   public EZGripperActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new EZGripperActionState(id, rootNode.getState()), rootNode);

      for (RobotSide side : RobotSide.values)
      {
         if (syncedRobot.getRobotModel().getRobotVersion().hasSakeGripperJoints(side))
         {
            x1KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(0));
            x2KnuckleJoints.put(side, (RevoluteJoint) syncedRobot.getFullRobotModel().getHand(side).getChildrenJoints().get(1));
            sakeHandStatus.put(side, syncedRobot.getSakeHandStatus().get(side));
         }
      }
   }

   @Override
   public void update()
   {
      super.update();

      trackingCalculator.update(Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp()));

      boolean hasX1KnuckleJoint = x1KnuckleJoints.get(definition.getSide()) != null;
      boolean hasX2KnuckleJoint = x2KnuckleJoints.get(definition.getSide()) != null;
      boolean isCalibrated = syncedRobot.getSakeHandStatus().get(definition.getSide()).getIsCalibrated();
      boolean needsReset = syncedRobot.getSakeHandStatus().get(definition.getSide()).getNeedsReset();

      boolean canExecute = hasX1KnuckleJoint;
      canExecute &= hasX2KnuckleJoint;
      canExecute &= isCalibrated;
      canExecute &= !needsReset;

      if (!canExecute)
         cantExecuteMessage = """
             Has X1 knuckle joint: %b
             Has X2 knuckle joint: %b
             Is calibrated: %b
             Needs reset: %b
             """.formatted(hasX1KnuckleJoint, hasX2KnuckleJoint, isCalibrated, needsReset);

      state.setCanExecute(canExecute);
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      trackingCalculator.reset();

      double goalKnuckleJointAngle = SakeHandParameters.handOpenAngleToKnuckleJointAngle(definition.getHandOpenAngle());
      trackingCalculator.addJointData(x1KnuckleJoints.get(definition.getSide()).getQ(), goalKnuckleJointAngle);
      trackingCalculator.addJointData(x2KnuckleJoints.get(definition.getSide()).getQ(), goalKnuckleJointAngle);
      trackingCalculator.factorInJointspaceErrors(definition.getInitialSatisfactionHandAngleTolerance());

      state.getLogger().info("x1: %.2f%s  x2: %.2f%s  Goal open angle: %.2f%s Error: %.2f%s"
                          .formatted(Math.toDegrees(x1KnuckleJoints.get(definition.getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(x2KnuckleJoints.get(definition.getSide()).getQ()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(definition.getHandOpenAngle()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL,
                                     Math.toDegrees(trackingCalculator.getTotalAbsoluteJointspaceError()),
                                     EuclidCoreMissingTools.DEGREE_SYMBOL));

      if (trackingCalculator.isWithinPositionTolerance())
      {
         state.getLogger().info("Gripper is already at the desired position. Proceeding to next action. (Error: %.2f)"
                             .formatted(trackingCalculator.getTotalAbsoluteJointspaceError()));
         state.setNominalExecutionDuration(0.0);
         state.setPositionDistanceToGoalTolerance(definition.getInitialSatisfactionHandAngleTolerance());
      }
      else
      {
         double handPositionLowerLimit = sakeHandStatus.get(definition.getSide()).getPositionLowerLimit();
         double handPositionUpperLimit = sakeHandStatus.get(definition.getSide()).getPositionUpperLimit();

         sakeHandDesiredCommandMessage.setRobotSide(definition.getSide().toByte());
         SakeHandParameters.resetDesiredCommandMessage(sakeHandDesiredCommandMessage);
         sakeHandDesiredCommandMessage.setGripperDesiredPosition(SakeHandParameters.handOpenAngleToPosition(definition.getHandOpenAngle(),
                                                                                                            handPositionLowerLimit,
                                                                                                            handPositionUpperLimit));
         sakeHandDesiredCommandMessage.setRawGripperTorqueLimit(SakeHandParameters.gripForceToRawTorque(definition.getFingertipGripForceLimit()));
         ros2ControllerHelper.publish(robotName -> SakeHandAPI.getHandSakeCommandTopic(robotName, definition.getSide()), sakeHandDesiredCommandMessage);

         state.getLogger().info("Commanding hand to open angle %.2f%s with torque limit %.2f N".formatted(Math.toDegrees(definition.getHandOpenAngle()),
                                                                                                 EuclidCoreMissingTools.DEGREE_SYMBOL,
                                                                                                 definition.getFingertipGripForceLimit()));

         state.getCommandedJointTrajectories().clear(2);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, x1KnuckleJoints.get(definition.getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(0, goalKnuckleJointAngle, NOMINAL_TRAJECTORY_DURATION);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, x2KnuckleJoints.get(definition.getSide()).getQ(), 0.0);
         state.getCommandedJointTrajectories().addTrajectoryPoint(1, goalKnuckleJointAngle, NOMINAL_TRAJECTORY_DURATION);
         state.setNominalExecutionDuration(NOMINAL_TRAJECTORY_DURATION);
         state.setPositionDistanceToGoalTolerance(definition.getCompletionHandAngleTolerance());
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      trackingCalculator.computeExecutionTimings(state.getNominalExecutionDuration());

      state.setElapsedExecutionTime(trackingCalculator.getElapsedTime());
      state.getCurrentJointAngles().setValue(0, x1KnuckleJoints.get(definition.getSide()).getQ());
      state.getCurrentJointAngles().setValue(1, x2KnuckleJoints.get(definition.getSide()).getQ());

      if (trackingCalculator.getHitTimeLimit(state.getLogger()))
      {
         state.setFailed(true);
         state.setIsExecuting(false);
      }
      else if (!state.getCommandedJointTrajectories().isEmpty())
      {
         trackingCalculator.resetJointspaceError();
         trackingCalculator.addJointData(x1KnuckleJoints.get(definition.getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(0).getPosition());
         trackingCalculator.addJointData(x2KnuckleJoints.get(definition.getSide()).getQ(),
                                         state.getCommandedJointTrajectories().getLastValueReadOnly(1).getPosition());
         trackingCalculator.factorInJointspaceErrors(state.getPositionDistanceToGoalTolerance());

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
