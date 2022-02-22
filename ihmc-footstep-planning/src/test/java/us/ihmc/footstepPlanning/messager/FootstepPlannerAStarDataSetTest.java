package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;

import java.util.Arrays;
import java.util.function.Predicate;

public class FootstepPlannerAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   protected boolean getPlanBodyPath()
   {
      return false;
   }

   @Override
   protected boolean getPerformAStarSearch()
   {
      return true;
   }

   @Override
   protected String getTestNamePrefix()
   {
      return "a_star";
   }

   @Override
   protected Predicate<PlannerInput> getTestableFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsTestable() && plannerInput.containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
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
//      MessagerAStarDataSetTest messagerAStarDataSetTest = new MessagerAStarDataSetTest();
//      messagerAStarDataSetTest.setup();
//      messagerAStarDataSetTest.testDataSets();

//      20190219_182005_Wall
//      20171215_211034_DoorwayNoCeiling

      FootstepPlannerAStarDataSetTest test = new FootstepPlannerAStarDataSetTest();
      DataSetName dataSetName = DataSetName._20171215_211034_DoorwayNoCeiling;
      test.testDataSets(Arrays.asList(DataSetIOTools.loadDataSet(dataSetName)));
   }
}
