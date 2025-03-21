package us.ihmc.alexander.posePlayback;

import us.ihmc.alexander.AlexanderVersion;
import us.ihmc.alexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.posePlayback.PlaybackPoseInterpolatorDRCTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AlexanderPlaybackPoseInterpolatorTest extends PlaybackPoseInterpolatorDRCTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new OpenAlexanderRobotModel(AlexanderVersion.V0_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }
}
