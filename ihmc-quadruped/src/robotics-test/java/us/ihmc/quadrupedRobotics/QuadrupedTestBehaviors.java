package us.ihmc.quadrupedRobotics;

import ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.EuclideanTrajectoryPointMessagePubSubType;
import quadruped_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.quadrupedBasics.QuadrupedSteppingStateEnum;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;

public class QuadrupedTestBehaviors
{
   private static double stateCompletionSafetyFactory = 1.3;

   public static void standUp(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables)
   {
      variables.getUserTrigger().set(HighLevelControllerName.FREEZE_STATE);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), HighLevelControllerName.FREEZE_STATE));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.25));
      conductor.simulate();

      variables.getUserTrigger().set(HighLevelControllerName.STAND_PREP_STATE);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), HighLevelControllerName.STAND_READY));
      conductor.addTimeLimit(variables.getYoTime(), stateCompletionSafetyFactory * variables.getTimeInStandPrep());
      conductor.simulate();

      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), 0.5));
      conductor.simulate();
   }

   public static void sitDown(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables)
   {
      variables.getUserTrigger().set(HighLevelControllerName.STAND_READY);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), HighLevelControllerName.STAND_READY));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.25));
      conductor.simulate();

      variables.getUserTrigger().set(QuadrupedControllerManager.sitDownStateName);
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), HighLevelControllerName.FREEZE_STATE));
      conductor.addTimeLimit(variables.getYoTime(), stateCompletionSafetyFactory * variables.getTimeToMoveSittingDown());
      conductor.simulate();
   }

   public static void readyXGait(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables, RemoteQuadrupedTeleopManager stepTeleopManager)
   {
      standUp(conductor, variables);
      startBalancing(conductor, variables, stepTeleopManager);
      squareUp(conductor, variables, stepTeleopManager);
   }

   public static void startBalancing(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables, RemoteQuadrupedTeleopManager teleopManager)
   {
      teleopManager.requestWalkingState();
      conductor.addTerminalGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.bodyHeight(variables, 0.1));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getControllerState(), HighLevelControllerName.WALKING));
      conductor.addTerminalGoal(YoVariableTestGoal.enumEquals(variables.getSteppingState(), QuadrupedSteppingStateEnum.STAND));
      conductor.addTimeLimit(variables.getYoTime(), stateCompletionSafetyFactory * variables.getToWalkingTransitionDuration());
      conductor.simulate();
   }

   public static void squareUp(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables, RemoteQuadrupedTeleopManager stepTeleopManager)
   {
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.2));
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.simulate();

      double initialEndPhaseShift = stepTeleopManager.getXGaitSettings().getEndPhaseShift();

      stepTeleopManager.setEndPhaseShift(180.0);
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(0.0, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.5));
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addSustainGoal(QuadrupedTestGoals.bodyHeight(variables, 0.25));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 1.0));
      conductor.simulate();

      stepTeleopManager.setEndPhaseShift(initialEndPhaseShift);
   }

   private static YoVariableTestGoal createBodyPathWaypointGoal(QuadrupedTestYoVariables variables, EuclideanTrajectoryPointMessage point, double positionDelta, double yawDelta)
   {
      double initialTime = variables.getYoTime().getDoubleValue();
      double timeDelta = 0.1;

      return new YoVariableTestGoal(variables.getYoTime())
      {
         @Override
         public boolean currentlyMeetsGoal()
         {
            return Math.abs(variables.getRobotBodyX().getDoubleValue() - point.position_.getX()) < positionDelta
                  && Math.abs(variables.getRobotBodyY().getDoubleValue() - point.position_.getY()) < positionDelta
                  && Math.abs(variables.getRobotBodyYaw().getDoubleValue() - point.position_.getZ()) < yawDelta
                  && Math.abs((variables.getYoTime().getDoubleValue() - initialTime) - point.getTime()) < timeDelta;
         }

         @Override
         public String toString()
         {
            return "Body path waypoint: " + point;
         }
      };
   }

   public static void executeBodyPathPlan(GoalOrientedTestConductor conductor, QuadrupedTestYoVariables variables,
                                          RemoteQuadrupedTeleopManager stepTeleopManager, double positionDelta, double yawDelta,
                                          EuclideanTrajectoryPointMessage... points)
   {
      QuadrupedBodyPathPlanMessage bodyPathPlanMessage = new QuadrupedBodyPathPlanMessage();
      bodyPathPlanMessage.setIsExpressedInAbsoluteTime(false);
      Object<EuclideanTrajectoryPointMessage> bodyPathPoints = new Object<> (4, EuclideanTrajectoryPointMessage.class, new EuclideanTrajectoryPointMessagePubSubType());

      for (int i = 0; i < points.length; i++)
      {
         EuclideanTrajectoryPointMessage point = points[i];
         bodyPathPoints.add().set(point);
         conductor.addWaypointGoal(createBodyPathWaypointGoal(variables, point, positionDelta, yawDelta));
      }
      bodyPathPlanMessage.body_path_points_ = bodyPathPoints;

      double trajectoryEndTime = points[points.length - 1].getTime() + 0.05;
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, trajectoryEndTime));

      stepTeleopManager.submitBodyPathPlan(bodyPathPlanMessage);
      stepTeleopManager.requestXGait();
      conductor.simulate();

      stepTeleopManager.requestStanding();
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.5));
      conductor.simulate();
   }
}
