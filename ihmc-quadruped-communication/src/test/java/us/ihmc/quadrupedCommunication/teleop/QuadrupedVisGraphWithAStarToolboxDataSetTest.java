package us.ihmc.quadrupedCommunication.teleop;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;

public class QuadrupedVisGraphWithAStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.VIS_GRAPH_WITH_A_STAR;
   }

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);
      settings.setEndDoubleSupportDuration(0.15);
//      settings.setEndDoubleSupportDuration(0.5 + 0.25);
      settings.setStepDuration(0.33);
//      settings.setEndPhaseShift(90);
      settings.setEndPhaseShift(180.0);
      return settings;
   }

   @Override
   @Test
   public void testDatasetsWithoutOcclusion()
   {
      super.testDatasetsWithoutOcclusion();
   }

   public static void main(String[] args) throws Exception
   {
      QuadrupedVisGraphWithAStarToolboxDataSetTest test = new QuadrupedVisGraphWithAStarToolboxDataSetTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171218_204953_FlatGroundWithWall");
//      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171216_111326_CrossoverPlatforms");
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
