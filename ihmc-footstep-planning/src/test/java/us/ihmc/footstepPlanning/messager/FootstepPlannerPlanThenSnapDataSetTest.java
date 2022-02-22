package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;

import java.util.Arrays;
import java.util.function.Predicate;

@Tag("footstep-planning-slow")
public class FootstepPlannerPlanThenSnapDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   protected boolean getPlanBodyPath()
   {
      return false;
   }

   @Override
   protected boolean getPerformAStarSearch()
   {
      return false;
   }

   @Override
   protected String getTestNamePrefix()
   {
      return "plan_then_snap";
   }

   private String getStatusFlag()
   {
      return "plan_then_snap_status";
   }

   @Override
   protected Predicate<PlannerInput> getTestableFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsTestable() && plannerInput.containsFlag(getStatusFlag()) && plannerInput.getAdditionalData(getStatusFlag()).get(0).equals("test");
   }

   @Override
   protected Predicate<PlannerInput> getInDevelopmentFilter()
   {
      return plannerInput -> plannerInput.getStepPlannerIsInDevelopment() && plannerInput.containsFlag(getStatusFlag());
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
      FootstepPlannerPlanThenSnapDataSetTest test = new FootstepPlannerPlanThenSnapDataSetTest();
      DataSetName dataSetName = DataSetName._20190219_182005_SteppingStones;
      test.testDataSets(Arrays.asList(DataSetIOTools.loadDataSet(dataSetName)));
   }
}
