package us.ihmc.footstepPlanning;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.function.Consumer;

public class FootstepPlanningModuleTest
{
   @Test
   public void testStreamingOutput()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190219_182005_Random);
      PlannerInput plannerInput = dataSet.getPlannerInput();

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(3.5);
      Pose3D initialMidFootPose = new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);

      // goal is unreachable
      Pose3D goalPose = new Pose3D(500.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);
      planningModule.getFootstepPlannerParameters().setReturnBestEffortPlan(true);

      // disable wiggling. causes latency of around 0.15s
      planningModule.getFootstepPlannerParameters().setMaximumXYWiggleDistance(0.0);
      planningModule.getFootstepPlannerParameters().setMaximumYawWiggle(0.0);

      Stopwatch stopwatch = new Stopwatch();
      double publishPeriod = 1.0;
      planningModule.setStatusPublishPeriod(publishPeriod);

      MutableInt numberOfStatusesReceived = new MutableInt();
      Consumer<FootstepPlannerOutput> streamingTester = output ->
      {
         if(output.getResult() == FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL)
         {
            double lapElapsed = stopwatch.lapElapsed();
            Assertions.assertTrue(MathTools.epsilonEquals(lapElapsed, publishPeriod, 0.08),
                                  "Planner doesn't appear to be streaming at the correct rate. Requested period: " + publishPeriod + ", actual: " + lapElapsed);

            stopwatch.lap();
            numberOfStatusesReceived.increment();
         }
      };

      planningModule.addStatusCallback(streamingTester);

      stopwatch.start();
      planningModule.handleRequest(request);
      double totalElapsed = stopwatch.totalElapsed();

      int expectedStatuses = (int) (totalElapsed / publishPeriod);
      Assertions.assertTrue(numberOfStatusesReceived.intValue() == expectedStatuses,
                            "Planner doesn't appear to be streaming correctly. Planning duration=" + totalElapsed + ", publish period=" + publishPeriod
                            + ", # of statuses=" + numberOfStatusesReceived.getValue());
   }

   @Test
   @Disabled
   public void testGoalProximity()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      planningModule.getFootstepPlannerParameters().setReturnBestEffortPlan(true);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D();
      Pose3D goalMidFootPose = new Pose3D(3.5, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setGoalDistanceProximity(0.65);
      request.setGoalYawProximity(0.4);
      request.setTimeout(2.0);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getResult().validForExecution());
   }

   @Test
   public void testRequestSnapGoalSteps()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

      double groundHeight = 2.5;
      double providedGoalNodeHeights = -1.0;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(0.0, 0.0, groundHeight, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(2.0, 0.0, providedGoalNodeHeights, 0.0, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setTimeout(2.0);

      // test snap goal steps
      request.setSnapGoalSteps(true);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getResult().validForExecution());
      FootstepPlan footstepPlan = plannerOutput.getFootstepPlan();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         boolean stepIsAtCorrectHeight = MathTools.epsilonEquals(footstepPlan.getFootstep(i).getSoleFramePose().getZ(), groundHeight, 1e-10);
         Assertions.assertTrue(stepIsAtCorrectHeight);
      }

      // test don't snap goal steps and abort if invalid
      goalMidFootPose.setPosition(100.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);

      request.setSnapGoalSteps(true);
      request.setAbortIfGoalStepSnappingFails(true);

      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getResult() == FootstepPlanningResult.INVALID_GOAL);

      // test that not snapping keeps original requested pose
      double heightOffset = 0.035;
      double rollOffset = -0.2;
      goalMidFootPose.setPosition(2.0, 0.0, groundHeight + heightOffset);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.getGoalFootPoses().forEach(pose -> pose.appendRollRotation(rollOffset));

      request.setSnapGoalSteps(false);

      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getResult().validForExecution());

      int planSize = plannerOutput.getFootstepPlan().getNumberOfSteps();
      for (int i = 0; i < 2; i++)
      {
         SimpleFootstep footstep = plannerOutput.getFootstepPlan().getFootstep(planSize - 1 - i);
         RobotSide robotSide = footstep.getRobotSide();
         boolean stepWasntAdjusted = footstep.getSoleFramePose().epsilonEquals(request.getGoalFootPoses().get(robotSide), 1e-10);
         Assertions.assertTrue(stepWasntAdjusted);
      }
   }
}
