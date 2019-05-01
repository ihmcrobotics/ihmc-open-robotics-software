package us.ihmc.quadrupedCommunication.teleop;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedPlanning.QuadrupedGait;
import us.ihmc.quadrupedPlanning.QuadrupedSpeed;
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
      settings.setStanceLength(0.9);
      settings.setStanceWidth(0.5);

      settings.getAmbleMediumTimings().setEndDoubleSupportDuration(0.5 + 0.25);
      settings.getAmbleMediumTimings().setStepDuration(0.5);
      settings.getAmbleFastTimings().setEndDoubleSupportDuration(0.25);
      settings.getAmbleFastTimings().setStepDuration(0.35);
      settings.getAmbleFastTimings().setMaxSpeed(1.0);
      settings.getTrotMediumTimings().setEndDoubleSupportDuration(0.1);
      settings.getTrotMediumTimings().setStepDuration(0.35);

      settings.setQuadrupedSpeed(QuadrupedSpeed.FAST);
      settings.setEndPhaseShift(QuadrupedGait.AMBLE.getEndPhaseShift());
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
      QuadrupedAStarToolboxDataSetTest test = new QuadrupedAStarToolboxDataSetTest();
      VISUALIZE = true;
      test.setup();

      test.runAssertions(DataSetName._20171115_171243_SimplePlaneAndWall);
      ThreadTools.sleepForever();
      test.tearDown();
   }
}
