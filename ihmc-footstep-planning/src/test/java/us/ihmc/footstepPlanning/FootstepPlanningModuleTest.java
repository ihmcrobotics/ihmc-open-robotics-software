package us.ihmc.footstepPlanning;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Assertions;
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
      request.setStatusPublishPeriod(publishPeriod);

      MutableInt numberOfStreamingStatuses = new MutableInt();

      Consumer<FootstepPlannerOutput> streamingTester = output ->
      {
         if(output.getFootstepPlanningResult() == FootstepPlanningResult.PLANNING)
         {
            // first status received is when body path planning is done and step planning starts
            if (numberOfStreamingStatuses.getValue() == 0)
            {
               stopwatch.start();
            }
            else
            {
               double lapElapsed = stopwatch.lap();
               Assertions.assertTrue(MathTools.epsilonEquals(lapElapsed, publishPeriod, 0.08),
                                     "Planner doesn't appear to be streaming at the correct rate. Requested period: " + publishPeriod + ", actual: " + lapElapsed);
            }

            numberOfStreamingStatuses.increment();
         }
      };

      planningModule.addStatusCallback(streamingTester);
      planningModule.handleRequest(request);
      double totalElapsed = stopwatch.totalElapsed();
      int numberOfStreamingIntervals = numberOfStreamingStatuses.getValue() - 1;

      int expectedStatuses = (int) (totalElapsed / publishPeriod);
      Assertions.assertTrue(numberOfStreamingIntervals == expectedStatuses,
                            "Planner doesn't appear to be streaming correctly. Planning duration=" + totalElapsed + ", publish period=" + publishPeriod
                            + ", # of statuses=" + numberOfStreamingStatuses.getValue());
   }
   
   @Test
   public void testGoalProximityWhenGoalIsUnreachable()
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
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
   }

   @Test
   public void testGoalProximityWhenGoalIsReachable()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      planningModule.getFootstepPlannerParameters().setReturnBestEffortPlan(true);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D();
      Pose3D goalMidFootPose = new Pose3D(2.0, 0.0, 0.0, 0.5 * Math.PI, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setGoalDistanceProximity(0.3);
      request.setGoalYawProximity(0.25 * Math.PI);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(200);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
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
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
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
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult() == FootstepPlanningResult.INVALID_GOAL);

      // test that not snapping keeps original requested pose
      double heightOffset = 0.035;
      double rollOffset = -0.2;
      goalMidFootPose.setPosition(2.0, 0.0, groundHeight + heightOffset);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.getGoalFootPoses().forEach(pose -> pose.appendRollRotation(rollOffset));

      request.setSnapGoalSteps(false);

      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());

      int planSize = plannerOutput.getFootstepPlan().getNumberOfSteps();
      for (int i = 0; i < 2; i++)
      {
         SimpleFootstep footstep = plannerOutput.getFootstepPlan().getFootstep(planSize - 1 - i);
         RobotSide robotSide = footstep.getRobotSide();
         boolean stepWasntAdjusted = footstep.getSoleFramePose().epsilonEquals(request.getGoalFootPoses().get(robotSide), 1e-10);
         Assertions.assertTrue(stepWasntAdjusted);
      }
   }

   @Test
   public void testPathHeading()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D();
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setTimeout(2.0);

      // test shuffling left
      Pose3D goalMidFootPose = new Pose3D(0.0, 1.25, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setDesiredHeading(FootstepPlanHeading.LEFT);
      request.setRequestedInitialStanceSide(RobotSide.RIGHT);
      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
      FootstepPlan plan = plannerOutput.getFootstepPlan();
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         double yaw = plan.getFootstep(i).getSoleFramePose().getYaw();
         double yawThreshold = Math.toRadians(25.0);
         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
      }

      // test shuffling right
      goalMidFootPose.set(0.0, -1.25, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setDesiredHeading(FootstepPlanHeading.RIGHT);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
      plan = plannerOutput.getFootstepPlan();
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         double yaw = plan.getFootstep(i).getSoleFramePose().getYaw();
         double yawThreshold = Math.toRadians(25.0);
         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
      }

      // test walking backward
      goalMidFootPose.set(-1.25, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setDesiredHeading(FootstepPlanHeading.BACKWARD);
      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
      plan = plannerOutput.getFootstepPlan();
      for (int i = 0; i < plan.getNumberOfSteps(); i++)
      {
         double yaw = plan.getFootstep(i).getSoleFramePose().getYaw();
         double yawThreshold = Math.toRadians(25.0);
         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
      }
   }
}
