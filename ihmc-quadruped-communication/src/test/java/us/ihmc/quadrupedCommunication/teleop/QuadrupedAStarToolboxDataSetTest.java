package us.ihmc.quadrupedCommunication.teleop;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;

public class QuadrupedAStarToolboxDataSetTest extends FootstepPlannerToolboxDataSetTest
{
   public FootstepPlannerType getPlannerType()
   {
      return FootstepPlannerType.A_STAR;
   }

   public QuadrupedXGaitSettingsReadOnly getXGaitSettings()
   {
      QuadrupedXGaitSettings settings = new QuadrupedXGaitSettings();
      settings.setStanceLength(1.0);
      settings.setStanceWidth(0.5);
      settings.setEndDoubleSupportDuration(0.25);
      settings.setStepDuration(0.5);
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
      QuadrupedAStarToolboxDataSetTest test = new QuadrupedAStarToolboxDataSetTest();
      String prefix = "unitTestDataSets/test/";
      VISUALIZE = true;
      test.setup();
      test.runAssertionsOnDataset(dataset -> test.runAssertions(dataset), "20171215_220523_SteppingStones");
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
