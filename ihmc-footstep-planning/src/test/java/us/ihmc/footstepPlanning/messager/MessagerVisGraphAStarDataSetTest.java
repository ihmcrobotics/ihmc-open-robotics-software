package us.ihmc.footstepPlanning.messager;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.footstepPlanning.FootstepPlannerDataSetTest;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;

import java.util.List;

public class MessagerVisGraphAStarDataSetTest extends FootstepPlannerDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataSet ->
                                                           {
                                                              if(!dataSet.hasPlannerInput())
                                                                 return false;
                                                              if(!dataSet.getPlannerInput().getStepPlannerIsTestable())
                                                                 return false;
                                                              if (dataSet.getPlannerInput().getVisGraphIsInDevelopment())
                                                                 return false;
                                                              return dataSet.getPlannerInput().containsTimeoutFlag(getPlannerType().toString().toLowerCase());
                                                           });
      super.runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @Override
   @Test
   @Disabled
   public void testDatasetsWithoutOcclusionInDevelopment()
   {
      super.testDatasetsWithoutOcclusionInDevelopment();
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
