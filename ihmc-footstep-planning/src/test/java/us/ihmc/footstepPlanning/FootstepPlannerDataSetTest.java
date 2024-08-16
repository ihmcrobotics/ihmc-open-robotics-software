package us.ihmc.footstepPlanning;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

public abstract class FootstepPlannerDataSetTest
{
   private static final boolean DEBUG = true;
   private static final boolean VERBOSE = true;
   private static boolean GENERATE_LOG_FOR_FAILING_TESTS = true;

   private final FootstepPlanningModule planningModule = new FootstepPlanningModule("testModule");
   private final FootstepPlannerLogger logger = new FootstepPlannerLogger(planningModule);

   protected abstract boolean getPlanBodyPath();

   protected abstract boolean getPerformAStarSearch();

   protected abstract String getTestNamePrefix();

   protected abstract Predicate<PlannerInput> getTestableFilter();

   protected abstract Predicate<PlannerInput> getInDevelopmentFilter();

   @BeforeEach
   public void setup()
   {
      GENERATE_LOG_FOR_FAILING_TESTS &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test
   public void testDataSets()
   {
      testDataSets(DataSetIOTools.loadDataSets(buildFilter(getTestableFilter())));
   }

   @Test
   @Disabled
   public void testDatasetsInDevelopment()
   {
      testDataSets(DataSetIOTools.loadDataSets(buildFilter(getInDevelopmentFilter())));
   }

   static Predicate<DataSet> buildFilter(Predicate<PlannerInput> testSpecificFilter)
   {
      return dataSet -> dataSet.hasPlannerInput() && testSpecificFilter.test(dataSet.getPlannerInput());
   }

   protected void testDataSets(List<DataSet> allDatasets)
   {
      if (VERBOSE || DEBUG)
         LogTools.info("Unit test files found: " + allDatasets.size());

      if (allDatasets.isEmpty())
         Assert.fail("Did not find any datasets to test.");

      int numberOfFailingTests = 0;
      List<String> failingDatasets = new ArrayList<>();
      int numberOfTestedDataSets = 0;
      for (int i = 0; i < allDatasets.size(); i++)
      {
         DataSet dataset = allDatasets.get(i);
         if (DEBUG || VERBOSE)
            LogTools.info("Testing file: " + dataset.getName());

         numberOfTestedDataSets++;
         String errorMessagesForCurrentFile = planAndCheckResult(dataset);
         if (!errorMessagesForCurrentFile.isEmpty())
         {
            numberOfFailingTests++;
            failingDatasets.add(dataset.getName());

            if (GENERATE_LOG_FOR_FAILING_TESTS)
            {
               logger.logSession();
            }
         }

         if (DEBUG || VERBOSE)
         {
            String result = errorMessagesForCurrentFile.isEmpty() ? "passed" : "failed";
            LogTools.info(dataset.getName() + " " + result);
         }
      }

      String message = "Number of failing datasets: " + numberOfFailingTests + " out of " + numberOfTestedDataSets;
      message += "\n Datasets failing: ";
      for (int i = 0; i < failingDatasets.size(); i++)
      {
         message += "\n" + failingDatasets.get(i);
      }

      Assert.assertEquals(message, 0, numberOfFailingTests);
   }

   private String planAndCheckResult(DataSet dataset)
   {
      FootstepPlannerRequest planningRequest = createPlanningRequest(dataset);
      FootstepPlannerOutput output = planningModule.handleRequest(planningRequest);

      String errorMessage = "";

      if(!output.getFootstepPlanningResult().validForExecution())
      {
         errorMessage = "Planning result for " + dataset.getName() + " is invalid, result was " + output.getFootstepPlanningResult();
      }
      else if (!PlannerTools.isGoalNextToLastStep(dataset.getPlannerInput().getGoalPosition(), output.getFootstepPlan()))
      {
         errorMessage = dataset.getName() + " did not reach goal. Made it to " + PlannerTools.getEndPosition(output.getFootstepPlan()) + ", trying to get to "
                        + dataset.getPlannerInput().getGoalPosition();
      }

      if(DEBUG && !errorMessage.isEmpty())
         LogTools.error(errorMessage);

      return errorMessage;
   }

   private FootstepPlannerRequest createPlanningRequest(DataSet dataset)
   {
      PlannerInput plannerInput = dataset.getPlannerInput();
      FootstepPlannerRequest request = new FootstepPlannerRequest();

      double startYaw = plannerInput.hasStartOrientation() ? plannerInput.getStartYaw() : 0.0;
      double goalYaw = plannerInput.hasGoalOrientation() ? plannerInput.getGoalYaw() : 0.0;
      SideDependentList<Pose3D> startSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getStartPosition(),
                                                                                   startYaw,
                                                                                   planningModule.getFootstepPlannerParameters().getIdealFootstepWidth());
      SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(plannerInput.getGoalPosition(),
                                                                                  goalYaw,
                                                                                  planningModule.getFootstepPlannerParameters().getIdealFootstepWidth());

      request.setStartFootPoses(startSteps.get(RobotSide.LEFT), startSteps.get(RobotSide.RIGHT));
      request.setGoalFootPoses(goalSteps.get(RobotSide.LEFT), goalSteps.get(RobotSide.RIGHT));
      request.setPlanBodyPath(getPlanBodyPath());
      request.setPerformAStarSearch(getPerformAStarSearch());
      request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(dataset.getPlanarRegionsList())));
      request.setMaximumIterations(300);
      request.setTimeout(Double.MAX_VALUE);
      request.setHorizonLength(Double.MAX_VALUE);
      return request;
   }
}
