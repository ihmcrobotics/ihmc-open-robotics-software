package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;

import java.util.Arrays;
import java.util.function.Predicate;

public class FootstepPlannerBodyPathAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   protected boolean getPlanBodyPath()
   {
      return true;
   }

   @Override
   protected boolean getPerformAStarSearch()
   {
      return true;
   }

   @Override
   protected String getTestNamePrefix()
   {
      return "vis_graph_with_a_star";
   }

   @Override
   protected Predicate<PlannerInput> getTestableFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsTestable() && !plannerInput.getVisGraphIsInDevelopment() && plannerInput.containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
   }

   @Override
   protected Predicate<PlannerInput> getInDevelopmentFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsInDevelopment() && plannerInput.containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
   }

   @Override
   @Test
   public void testDataSets()
   {
      super.testDataSets();
   }

   @Override
   @Test
   @Disabled
   public void testDatasetsInDevelopment()
   {
      super.testDatasetsInDevelopment();
   }

   public static void main(String[] args) throws Exception
   {
      FootstepPlannerBodyPathAStarDataSetTest test = new FootstepPlannerBodyPathAStarDataSetTest();
      DataSetName dataSetName = DataSetName._20171026_131304_PlanarRegion_Ramp_2Story_UnitTest;
      test.testDataSets(Arrays.asList(DataSetIOTools.loadDataSet(dataSetName)));
   }
}
