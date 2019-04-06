package us.ihmc.atlas.posePlayback;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.posePlayback.PlaybackPoseSequenceDRCTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasPlaybackPoseSequenceTest extends PlaybackPoseSequenceDRCTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @Test
   public void testReadAndWriteWithRandomSequence()
   {
      super.testReadAndWriteWithRandomSequence();
   }

}
