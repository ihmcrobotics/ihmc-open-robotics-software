package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pubsub.DomainFactory;

import java.util.List;

public class VisGraphWithAStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   @Override
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   @Override
   @Test
   public void testDataSets()
   {
      List<DataSet> dataSets = DataSetIOTools.loadDataSets(dataset ->
                                                           {
                                                              if (!dataset.hasPlannerInput())
                                                                 return false;
                                                              if (!dataset.getPlannerInput().getStepPlannerIsTestable())
                                                                 return false;
                                                              if (dataset.getPlannerInput().getVisGraphIsInDevelopment())
                                                                 return false;
                                                              return dataset.getPlannerInput().containsFlag(getTimeoutFlag());
                                                           });
      runAssertionsOnAllDatasets(this::runAssertions, dataSets);
   }

   @Override
   @Test
   @Disabled
   public void runInDevelopmentDataSets()
   {
      super.runInDevelopmentDataSets();
   }

   public static void main(String[] args) throws Exception
   {
      VisGraphWithAStarToolboxDataSetTest test = new VisGraphWithAStarToolboxDataSetTest();

      test.pubSubImplementation = DomainFactory.PubSubImplementation.INTRAPROCESS;
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(test::runAssertions, DataSetName._20171215_214730_CinderBlockField);

      ThreadTools.sleepForever();
      test.tearDown();
      PrintTools.info("Test passed.");
   }
}
