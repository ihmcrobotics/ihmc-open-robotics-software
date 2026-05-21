package us.ihmc.footstepPlanning;

import com.google.common.util.concurrent.AtomicDouble;
import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import perception_msgs.HeightMapMessage;
import perception_msgs.TerrainMapMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class FootstepPlanningModuleTest
{
   @Test
   @Disabled // flaky - test is based on CPU time
   // TODO expose NowSupplier in Stopwatch or base planner timing on some manual time source for unit testing
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
      request.setPlanBodyPath(false);

      // goal is unreachable
      Pose3D goalPose = new Pose3D(500.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

      // disable wiggling. causes latency of around 0.15s
      planningModule.getFootstepPlannerParameters().setMaxXYWiggleDistance(0.0);
      planningModule.getFootstepPlannerParameters().setMaxYawWiggle(0.0);

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
      Assertions.assertEquals(numberOfStreamingIntervals,
                              expectedStatuses,
                              "Planner doesn't appear to be streaming correctly. Planning duration=" + totalElapsed + ", publish period=" + publishPeriod
                              + ", # of statuses=" + numberOfStreamingStatuses.getValue());
   }

   /**
    * This test is dependent on a lot of parameters. The ideal step length or ideal step yaw will determine where the step starts.
    * From there we can move it some amount but those parameters could cause the test to pass or fail. Same with the goal distance proximity.
    * So changing this guy: planningModule.getFootstepPlannerParameters().setIdealFootstepLength(0.2); can break the test
    */
   @Test
   public void testGoalProximityWhenGoalIsUnreachable()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D();
      // Since the support is only 6x6, a goal pose of 3.5 is out bounds. The bounds are 3.0.
      Pose3D goalMidFootPose = new Pose3D(3.5, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanBodyPath(false);
      request.setAssumeFlatGround(false);
      request.setGoalDistanceProximity(0.7); // This proximity means we should be ok. We might not check the value exactly x = 3.0, so (x + goalDistanceProximity)
      request.setTerrainMapData(TerrainMapMessageTools.unpackMessage(terrainMapMessage));
      request.setGoalYawProximity(0.4);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(50);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution(), "Valid for Execution was " + plannerOutput.getFootstepPlanningResult().validForExecution());
   }

   @Test
   public void testGoalProximityWhenGoalIsReachable()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      planningModule.getFootstepPlannerParameters().setMaxBranchFactor(0);
      planningModule.getFootstepPlannerParameters().setMaxStepYaw(0.25 * Math.PI);
      planningModule.getFootstepPlannerParameters().setMinStepYaw(-0.25 * Math.PI);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D();
      Pose3D goalMidFootPose = new Pose3D(2.0, 0.0, 0.0, 0.5 * Math.PI, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanBodyPath(false);
      request.setTerrainMapData(TerrainMapMessageTools.unpackMessage(terrainMapMessage));
      request.setGoalDistanceProximity(0.3);
      request.setGoalYawProximity(0.25 * Math.PI);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(2000);

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

      PlanarRegionsList planarRegionsList = planarRegionsListGenerator.getPlanarRegionsList();
      TerrainMapMessage terrainMapMessage = PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegionsList);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(0.0, 0.0, groundHeight, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(2.0, 0.0, providedGoalNodeHeights, 0.0, 0.0, 0.0);
      request.setTerrainMapData(TerrainMapMessageTools.unpackMessage(terrainMapMessage));
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanBodyPath(false);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(30);

      // test snap goal steps
      request.setSnapGoalSteps(true);
      // We need to set this false, otherwise it automatically sets the footstep height.
      request.setAssumeFlatGround(false);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
      FootstepPlan footstepPlan = plannerOutput.getFootstepPlan();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         boolean stepIsAtCorrectHeight = MathTools.epsilonEquals(footstepPlan.getFootstep(i).getFootstepPose().getZ(), groundHeight, 1e-10);
         Assertions.assertTrue(stepIsAtCorrectHeight);
      }

      // test don't snap goal steps and abort if invalid
      goalMidFootPose.getPosition().set(100.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);

      request.setSnapGoalSteps(true);
      request.setAbortIfGoalStepSnappingFails(true);

      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertEquals(FootstepPlanningResult.INVALID_GOAL, plannerOutput.getFootstepPlanningResult());

      // test that not snapping keeps original requested pose
      double heightOffset = 0.035;
      double rollOffset = -0.2;
      goalMidFootPose.getPosition().set(2.0, 0.0, groundHeight + heightOffset);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.getGoalFootPoses().forEach(pose -> pose.appendRollRotation(rollOffset));

      request.setSnapGoalSteps(false);

      plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());

      int planSize = plannerOutput.getFootstepPlan().getNumberOfSteps();
      for (int i = 0; i < 2; i++)
      {
         PlannedFootstep footstep = plannerOutput.getFootstepPlan().getFootstep(planSize - 1 - i);
         RobotSide robotSide = footstep.getRobotSide();
         boolean stepWasntAdjusted = footstep.getFootstepPose().epsilonEquals(request.getGoalFootPoses().get(robotSide), 1e-10);
         Assertions.assertTrue(stepWasntAdjusted);
      }
   }

//   @Test
//   public void testPathHeading()
//   {
//      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
//
//      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
//      planarRegionsListGenerator.addRectangle(6.0, 6.0);
//
//      FootstepPlannerRequest request = new FootstepPlannerRequest();
//      Pose3D initialMidFootPose = new Pose3D();
//      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
//      request.setRequestedInitialStanceSide(RobotSide.LEFT);
//      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
//      request.setPlanBodyPath(false);
//      request.setTimeout(2.0);
//
//      // test shuffling left
//      Pose3D goalMidFootPose = new Pose3D(0.0, 1.25, 0.0, 0.0, 0.0, 0.0);
//      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
//      request.setDesiredHeading( -0.5 * Math.PI);
//      request.setRequestedInitialStanceSide(RobotSide.RIGHT);
//      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
//      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
//      FootstepPlan plan = plannerOutput.getFootstepPlan();
//      for (int i = 0; i < plan.getNumberOfSteps(); i++)
//      {
//         double yaw = plan.getFootstep(i).getFootstepPose().getYaw();
//         double yawThreshold = Math.toRadians(25.0);
//         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
//      }
//
//      // test shuffling right
//      goalMidFootPose.set(0.0, -1.25, 0.0, 0.0, 0.0, 0.0);
//      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
//      request.setDesiredHeading(0.5 * Math.PI);
//      request.setRequestedInitialStanceSide(RobotSide.LEFT);
//      plannerOutput = planningModule.handleRequest(request);
//      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
//      plan = plannerOutput.getFootstepPlan();
//      for (int i = 0; i < plan.getNumberOfSteps(); i++)
//      {
//         double yaw = plan.getFootstep(i).getFootstepPose().getYaw();
//         double yawThreshold = Math.toRadians(25.0);
//         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
//      }
//
//      // test walking backward
//      goalMidFootPose.set(-1.25, 0.0, 0.0, 0.0, 0.0, 0.0);
//      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
//      request.setDesiredHeading(Math.PI);
//      plannerOutput = planningModule.handleRequest(request);
//      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
//      plan = plannerOutput.getFootstepPlan();
//      for (int i = 0; i < plan.getNumberOfSteps(); i++)
//      {
//         double yaw = plan.getFootstep(i).getFootstepPose().getYaw();
//         double yawThreshold = Math.toRadians(25.0);
//         Assertions.assertTrue(Math.abs(yaw) < Math.abs(yawThreshold));
//      }
//   }

   @Test
   public void testCustomTermination()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      DataSet dataSet = DataSetIOTools.loadDataSet(DataSetName._20190219_182005_Random);
      PlannerInput plannerInput = dataSet.getPlannerInput();

      // unreachable goal to make sure planner doesn't find plan
      Pose3D goalPose = new Pose3D(8.0, 0.0, 0.0, 0.0, 0.0, 0.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setTimeout(Double.MAX_VALUE);
      Pose3D initialMidFootPose = new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanBodyPath(false);
      request.setAbortIfBodyPathPlannerFails(false);
      request.setSnapGoalSteps(false);

      Stopwatch stopwatch = new Stopwatch();

      // test time
      double customTimeout = 1.0;
      AtomicDouble timestampPrev = new AtomicDouble();
      AtomicDouble timestamp = new AtomicDouble();
      AtomicBoolean firstTick = new AtomicBoolean(true);
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> time >= customTimeout);
      planningModule.addIterationCallback(iteration ->
                                          {
                                             if (firstTick.getAndSet(false))
                                             {
                                                stopwatch.start();

                                                // In order to reach the timeout value, we sleep that long
                                                ThreadTools.sleepSeconds(1.0);
                                             }

                                             timestampPrev.set(timestamp.get());
                                             timestamp.set(stopwatch.totalElapsed());
                                          });
      FootstepPlannerOutput output = planningModule.handleRequest(request);
      Assertions.assertEquals(FootstepPlanningResult.HALTED, output.getFootstepPlanningResult());
      Assertions.assertTrue(timestampPrev.get() < customTimeout);
      Assertions.assertTrue(output.getPlannerTimings().getTotalElapsedSeconds() >= customTimeout);

      // test iteration limit
      int iterationLimit = 29;
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> iterations >= iterationLimit);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(FootstepPlanningResult.HALTED, output.getFootstepPlanningResult());
      Assertions.assertEquals(iterationLimit, output.getPlannerTimings().getStepPlanningIterations());

      // test step limit
      int stepLimit = 4;
      request.setAssumeFlatGround(true);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> pathSize >= stepLimit);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(FootstepPlanningResult.HALTED, output.getFootstepPlanningResult());
      Assertions.assertEquals(stepLimit, output.getFootstepPlan().getNumberOfSteps());

      // test final step position
      double xThreshold = 3.88;
      request.setAssumeFlatGround(true);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> finalStep.getTranslationX() >= xThreshold);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(FootstepPlanningResult.HALTED, output.getFootstepPlanningResult());
      FootstepPlan plan = output.getFootstepPlan();

      double finalStepX = plan.getFootstep(plan.getNumberOfSteps() - 1).getFootstepPose().getX();
      Assertions.assertTrue(MathTools.intervalContains(finalStepX, xThreshold, xThreshold + planningModule.getFootstepPlannerParameters().getMaxStepReach(), 1e-5));
   }
}
