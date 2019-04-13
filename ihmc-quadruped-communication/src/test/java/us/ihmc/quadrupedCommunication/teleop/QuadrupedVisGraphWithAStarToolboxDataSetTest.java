package us.ihmc.quadrupedCommunication.teleop;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
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

      settings.getTrotMediumTimings().setStepDuration(0.33);
      settings.getTrotMediumTimings().setEndDoubleSupportDuration(0.15);
      settings.getAmbleMediumTimings().setStepDuration(0.4);
      settings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.5 + 0.25);

      settings.setQuadrupedSpeed(QuadrupedSpeed.MEDIUM);
      settings.setEndPhaseShift(QuadrupedGait.TROT.getEndPhaseShift());
      return settings;
   }

   @Override
   @Test
   public void testDataSets()
   {
      super.testDataSets();
   }

   public static void main(String[] args) throws Exception
   {
      QuadrupedVisGraphWithAStarToolboxDataSetTest test = new QuadrupedVisGraphWithAStarToolboxDataSetTest();
      VISUALIZE = true;
      test.setup();
      test.runAssertions(DataSetName._20171218_204953_FlatGroundWithWall);
//      test.runAssertions(DataSetName._20171216_111326_CrossoverPlatforms);
      ThreadTools.sleepForever();
      test.tearDown();

   }
}
