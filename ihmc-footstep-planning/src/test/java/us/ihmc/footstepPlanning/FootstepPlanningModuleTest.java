package us.ihmc.footstepPlanning;

import com.google.common.util.concurrent.AtomicDouble;
import geometry_msgs.Pose;
import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.geometry.PlanarRegionsListGenerator;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Random;
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
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);

      // goal is unreachable
      Pose3D goalPose = new Pose3D(500.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

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
      Assertions.assertEquals(numberOfStreamingIntervals,
                              expectedStatuses,
                              "Planner doesn't appear to be streaming correctly. Planning duration=" + totalElapsed + ", publish period=" + publishPeriod
                              + ", # of statuses=" + numberOfStreamingStatuses.getValue());
   }
   
   @Test
   public void testGoalProximityWhenGoalIsUnreachable()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

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
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(50);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getFootstepPlanningResult().validForExecution());
   }

   @Test
   public void testGoalProximityWhenGoalIsReachable()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      planningModule.getFootstepPlannerParameters().setMaximumBranchFactor(0);

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

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(0.0, 0.0, groundHeight, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(2.0, 0.0, providedGoalNodeHeights, 0.0, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(30);

      // test snap goal steps
      request.setSnapGoalSteps(true);

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
      Assertions.assertSame(plannerOutput.getFootstepPlanningResult(), FootstepPlanningResult.INVALID_GOAL);

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
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setAbortIfBodyPathPlannerFails(false);
      request.setSnapGoalSteps(false);

      Stopwatch stopwatch = new Stopwatch();

      // test time
      double customTimeout = 1.65;
      AtomicDouble timestampPrev = new AtomicDouble();
      AtomicDouble timestamp = new AtomicDouble();
      AtomicBoolean firstTick = new AtomicBoolean(true);
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> time >= customTimeout);
      planningModule.addIterationCallback(iteration ->
                                          {
                                             if (firstTick.getAndSet(false))
                                             {
                                                stopwatch.start();
                                             }

                                             timestampPrev.set(timestamp.get());
                                             timestamp.set(stopwatch.totalElapsed());
                                          });
      FootstepPlannerOutput output = planningModule.handleRequest(request);
      Assertions.assertEquals(output.getFootstepPlanningResult(), FootstepPlanningResult.HALTED);
      Assertions.assertTrue(timestampPrev.get() < customTimeout);
      Assertions.assertTrue(output.getPlannerTimings().getTotalElapsedSeconds() >= customTimeout);

      // test iteration limit
      int iterationLimit = 29;
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> iterations >= iterationLimit);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(output.getFootstepPlanningResult(), FootstepPlanningResult.HALTED);
      Assertions.assertEquals(output.getPlannerTimings().getStepPlanningIterations(), iterationLimit);

      // test step limit
      int stepLimit = 4;
      request.setAssumeFlatGround(true);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> pathSize >= stepLimit);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(output.getFootstepPlanningResult(), FootstepPlanningResult.HALTED);
      Assertions.assertEquals(output.getFootstepPlan().getNumberOfSteps(), stepLimit);

      // test final step position
      double xThreshold = 3.88;
      request.setAssumeFlatGround(true);
      planningModule.clearCustomTerminationConditions();
      planningModule.addCustomTerminationCondition((time, iterations, finalStep, secondToFinalStep, pathSize) -> finalStep.getTranslationX() >= xThreshold);
      output = planningModule.handleRequest(request);
      Assertions.assertEquals(output.getFootstepPlanningResult(), FootstepPlanningResult.HALTED);
      FootstepPlan plan = output.getFootstepPlan();

      double finalStepX = plan.getFootstep(plan.getNumberOfSteps() - 1).getFootstepPose().getX();
      Assertions.assertTrue(MathTools.intervalContains(finalStepX, xThreshold, xThreshold + planningModule.getFootstepPlannerParameters().getMaximumStepReach(), 1e-5));
   }

   @ Test
   public void testReferenceBasedAStarPlanner()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(planningModule);

      double groundHeight = 2.5;
      double providedGoalNodeHeights = 0.0;

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.translate(0.0, 0.0, groundHeight);
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      Pose3D initialMidFootPose = new Pose3D(0.0, 0.0, groundHeight, 0.0, 0.0, 0.0);
      Pose3D goalMidFootPose = new Pose3D(6.0, 6.0, providedGoalNodeHeights, 0.0, 0.0, 0.0);
      request.setStartFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), initialMidFootPose);
      request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPose);
      request.setRequestedInitialStanceSide(RobotSide.LEFT);
//      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(true);
      request.setTimeout(Double.MAX_VALUE);
      request.setMaximumIterations(500);
      request.setPerformAStarSearch(true);
      request.setAssumeFlatGround(true);

      // test snap goal steps
      request.setSnapGoalSteps(true);

      // set alpha for ref calc
      double referenceAlpha = 0.0;
      planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().setReferenceAlpha(referenceAlpha);

      ArrayList<Pose3D> goalMidFootPoseList = new ArrayList<>();

      Random random = new Random();
      double y = EuclidCoreRandomTools.nextDouble(random, 2,5);
      double x = EuclidCoreRandomTools.nextDouble(random, 2,5);
      double yaw = EuclidCoreRandomTools.nextDouble(random, 0, Math.PI / 4);

      goalMidFootPoseList.add(new Pose3D(x, y, providedGoalNodeHeights, yaw, 0, 0));

      ArrayList<String> data = new ArrayList<>();
      String msg;
      for (int i = 0; i < goalMidFootPoseList.size(); ++i)
      {
         // case: no referencePlan (nominal)
         request.setReferencePlan(null);
         request.setGoalFootPoses(planningModule.getFootstepPlannerParameters().getIdealFootstepWidth(), goalMidFootPoseList.get(i));
         FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
         footstepPlannerLogger.logSession();
         FootstepPlan originalPlan = plannerOutput.getFootstepPlan();

         msg = "noreference First two are left right goal pose ";
         data.add(msg);
         for (RobotSide robotSide : RobotSide.values)
         {
            Pose3D goalPose = request.getGoalFootPoses().get(robotSide);
            msg = "side " + robotSide.toString() + " x " + String.format("%.3f", goalPose.getX()) + " y " + String.format("%.3f", goalPose.getY()) + " yaw "
                  + String.format("%.3f", goalPose.getYaw());
            data.add(msg);
         }
         for (int j = 0; j < originalPlan.getNumberOfSteps(); ++j)
         {
            PlannedFootstep step = originalPlan.getFootstep(j);
            msg = "side " + step.getRobotSide() + " x " + String.format("%.3f", step.getFootstepPose().getX()) + " y " + String.format("%.3f",
                                                                                                                                         step.getFootstepPose()
                                                                                                                                             .getY()) + " yaw "
                  + String.format("%.3f", step.getFootstepPose().getYaw());
            data.add(msg);
         }
         // NOTE: Now perturb the original plan and make that as a reference plan to follow with same goal to simulate real world case
         FootstepPlan perturbedPlan = new FootstepPlan();
         for (int j = 0; j < originalPlan.getNumberOfSteps(); ++j)
         {
            PlannedFootstep step = originalPlan.getFootstep(j);
            // max 0.1[m] and 30 deg deviation from original step
            double change_in_x = EuclidCoreRandomTools.nextDouble(random, 0.1);
            double change_in_y = EuclidCoreRandomTools.nextDouble(random, 0.1);
            double change_in_yaw = EuclidCoreRandomTools.nextDouble(random, Math.PI/6);
            double x_new = step.getFootstepPose().getX() + change_in_x;
            double y_new = step.getFootstepPose().getY() + change_in_y;
            double yaw_new = step.getFootstepPose().getYaw() + change_in_yaw;
            Pose3D perturbedStepPose = new Pose3D(x_new, y_new, providedGoalNodeHeights, yaw_new, 0, 0);
            PlannedFootstep perturbedStep = new PlannedFootstep(step.getRobotSide(), perturbedStepPose);
            perturbedPlan.addFootstep(perturbedStep);
         }

         msg = "perturbed First two are left right goal pose ";
         data.add(msg);
         for (RobotSide robotSide : RobotSide.values)
         {
            Pose3D goalPose = request.getGoalFootPoses().get(robotSide);
            msg = "side " + robotSide.toString() + " x " + String.format("%.3f", goalPose.getX()) + " y " + String.format("%.3f", goalPose.getY()) + " yaw "
                  + String.format("%.3f", goalPose.getYaw());
            data.add(msg);
         }
         for (int j = 0; j < perturbedPlan.getNumberOfSteps(); ++j)
         {
            PlannedFootstep step = perturbedPlan.getFootstep(j);
            msg = "side " + step.getRobotSide() + " x " + String.format("%.3f", step.getFootstepPose().getX()) + " y " + String.format("%.3f",
                                                                                                                                       step.getFootstepPose()
                                                                                                                                           .getY()) + " yaw "
                  + String.format("%.3f", step.getFootstepPose().getYaw());
            data.add(msg);
         }
         // case: yes reference plan
         // use various reference alphas to see the effects
         request.setReferencePlan(perturbedPlan);
         for (referenceAlpha = 0.5; referenceAlpha <= 1.0; referenceAlpha+=1.0)
         {
            planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().setReferenceAlpha(referenceAlpha);
            plannerOutput = planningModule.handleRequest(request);
            footstepPlannerLogger.logSession();
            FootstepPlan referencedPlan = plannerOutput.getFootstepPlan();
            double currentAlpha = planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().getReferenceAlpha();
            msg = "yesreference alpha " + currentAlpha +" First two are left right goal pose";
            data.add(msg);
            for (RobotSide robotSide : RobotSide.values)
            {
               Pose3D goalPose = request.getGoalFootPoses().get(robotSide);
               msg = "side " + robotSide.toString() + " x " + String.format("%.3f", goalPose.getX()) + " y " + String.format("%.3f", goalPose.getY()) + " yaw "
                     + String.format("%.3f", goalPose.getYaw());
               data.add(msg);
            }
            for (int j = 0; j < referencedPlan.getNumberOfSteps(); ++j)
            {
               PlannedFootstep step = referencedPlan.getFootstep(j);
               msg = "side " + step.getRobotSide() + " x " + String.format("%.3f", step.getFootstepPose().getX()) + " y " + String.format("%.3f",
                                                                                                                                          step.getFootstepPose()
                                                                                                                                              .getY()) + " yaw "
                     + String.format("%.3f", step.getFootstepPose().getYaw());
               data.add(msg);
            }
         }

         try
         {
            FileWriter myWriter = new FileWriter("referencePlanSteps.txt");
            for (String message : data)
            {
               myWriter.write(message + "\n");
            }
            myWriter.close();
            System.out.println("Successfully wrote to the file.");
         }
         catch (IOException e)
         {
            System.out.println("An error occurred.");
            e.printStackTrace();
         }

         // for alpha = 0, new plan should be exactly the same as original
         planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().setReferenceAlpha(0.0);
         for (int j = 0; j < 10; ++j)
         {
            plannerOutput = planningModule.handleRequest(request);
            FootstepPlan referencedPlan_alpha_0 = plannerOutput.getFootstepPlan();
            Assertions.assertTrue(EuclidCoreTools.equals(originalPlan.getNumberOfSteps(), referencedPlan_alpha_0.getNumberOfSteps()));
            for (int k = 0; k < originalPlan.getNumberOfSteps(); ++k)
            {
               Assertions.assertTrue(EuclidCoreTools.equals(originalPlan.getFootstep(k).getFootstepPose(),
                                                            referencedPlan_alpha_0.getFootstep(k).getFootstepPose()));
            }
         }

         // TODO: for alpha = 1.0, new plan should be further away from original but does not mean it will be close to the perturbed(referenced) plan...
         //  how to test?
         request.setReferencePlan(perturbedPlan);
         planningModule.getAStarFootstepPlanner().getReferenceBasedIdealStepCalculator().setReferenceAlpha(1.0);
         plannerOutput = planningModule.handleRequest(request);
         FootstepPlan referencedPlan_alpha_1 = plannerOutput.getFootstepPlan();

      }

//      try
//      {
//         FileWriter myWriter = new FileWriter("referencePlanSteps.txt");
//         for (String message : data)
//         {
//            myWriter.write(message + "\n");
//         }
//         myWriter.close();
//         System.out.println("Successfully wrote to the file.");
//      }
//      catch (IOException e)
//      {
//         System.out.println("An error occurred.");
//         e.printStackTrace();
//      }
   }
}
