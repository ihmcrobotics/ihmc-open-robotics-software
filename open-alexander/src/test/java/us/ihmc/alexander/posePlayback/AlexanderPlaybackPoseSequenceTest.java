package us.ihmc.alexander.posePlayback;

import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.OpenAlexanderVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequenceDRCTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderPlaybackPoseSequenceTest extends PlaybackPoseSequenceDRCTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(OpenAlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   @Test
   public void testReadAndWriteWithRandomSequence()
   {
      super.testReadAndWriteWithRandomSequence();
   }
}
