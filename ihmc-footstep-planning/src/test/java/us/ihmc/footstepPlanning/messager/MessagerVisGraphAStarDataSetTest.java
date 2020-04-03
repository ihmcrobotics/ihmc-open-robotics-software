package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;

import java.util.List;

public class MessagerVisGraphAStarDataSetTest extends FootstepPlannerDataSetTest
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
   @Test
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if(!dataSet.getPlannerInput().getStepPlannerIsTestable())
                                                                 return false;
                                                              if (dataSet.getPlannerInput().getVisGraphIsInDevelopment())
                                                                 return false;
                                                              return dataSet.getPlannerInput().containsIterationLimitFlag(getTestNamePrefix().toLowerCase());
                                                           });
      super.runAssertionsOnAllDatasets(this::runAssertions, dataSets);
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
      MessagerVisGraphAStarDataSetTest test = new MessagerVisGraphAStarDataSetTest();
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20171026_131304_PlanarRegion_Ramp_2Story_UnitTest);
      ThreadTools.sleepForever();
      test.tearDown();
   }
}
