package us.ihmc.atlas.packets.wholebody;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.packetEndToEndTests.ArmJointTrajectoryPacketEndToEndTest;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;

@DeployableTestClass(planType = {BambooPlanType.Fast})
public class AtlasArmJointTrajectoryPacketTest extends ArmJointTrajectoryPacketEndToEndTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
         return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return null;
   }
}
