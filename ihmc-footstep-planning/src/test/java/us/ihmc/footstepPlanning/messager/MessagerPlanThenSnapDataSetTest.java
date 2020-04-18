package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;

import java.util.List;

@Tag("footstep-planning-slow")
public class MessagerPlanThenSnapDataSetTest extends FootstepPlannerDataSetTest
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

   @Override
   @Test
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if (!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if (!dataSet.getPlannerInput().getStepPlannerIsTestable())
                                                                 return false;
                                                              if (!dataSet.getPlannerInput().containsFlag(getStatusFlag()))
                                                                 return false;
                                                              return dataSet.getPlannerInput().getAdditionalData(getStatusFlag()).get(0).equals("test");
                                                           });

      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @Override
   @Test
   @Disabled
   public void testDatasetsInDevelopment()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if(!dataSet.getPlannerInput().getStepPlannerIsInDevelopment())
                                                                 return false;
                                                              if (!dataSet.getPlannerInput().containsFlag(getStatusFlag()))
                                                                 return false;
                                                              return true;
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   private String getStatusFlag()
   {
      return "plan_then_snap_status";
   }

   public static void main(String[] args) throws Exception
   {
      MessagerPlanThenSnapDataSetTest test = new MessagerPlanThenSnapDataSetTest();
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20190219_182005_SteppingStones);
      test.tearDown();
   }
}
