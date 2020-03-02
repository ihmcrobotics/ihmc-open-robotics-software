package us.ihmc.footstepPlanning;

import org.apache.commons.lang3.mutable.MutableInt;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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
      request.setInitialStancePose(new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0)));
      request.setInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(dataSet.getPlanarRegionsList());
      request.setPlanBodyPath(false);

      // goal is unreachable
      Pose3D goalPose = new Pose3D(500.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      request.setGoalPose(goalPose);
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
   public void testGoalProximity()
   {
      FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());
      planningModule.getFootstepPlannerParameters().setReturnBestEffortPlan(true);

      PlanarRegionsListGenerator planarRegionsListGenerator = new PlanarRegionsListGenerator();
      planarRegionsListGenerator.addRectangle(6.0, 6.0);

      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setGoalPose(new Pose3D(3.5, 0.0, 0.0, 0.0, 0.0, 0.0));
      request.setInitialStancePose(new Pose3D());
      request.setInitialStanceSide(RobotSide.LEFT);
      request.setPlanarRegionsList(planarRegionsListGenerator.getPlanarRegionsList());
      request.setPlanBodyPath(false);
      request.setGoalDistanceProximity(0.65);
      request.setGoalYawProximity(0.4);
      request.setTimeout(2.0);

      FootstepPlannerOutput plannerOutput = planningModule.handleRequest(request);
      Assertions.assertTrue(plannerOutput.getResult().validForExecution());
   }
}
