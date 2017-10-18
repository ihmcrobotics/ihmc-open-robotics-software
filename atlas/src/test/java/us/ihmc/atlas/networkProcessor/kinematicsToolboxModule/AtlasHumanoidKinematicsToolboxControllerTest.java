package us.ihmc.atlas.networkProcessor.kinematicsToolboxModule;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.AvatarHumanoidKinematicsToolboxControllerTest;

public class AtlasHumanoidKinematicsToolboxControllerTest extends AvatarHumanoidKinematicsToolboxControllerTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   private DRCRobotModel ghostRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public void testHoldBodyPose() throws Exception
   {
      super.testHoldBodyPose();
   }

   @Override
   public void testRandomHandPositions() throws Exception
   {
      super.testRandomHandPositions();
   }

   @Override
   public void testRandomHandPoses() throws Exception
   {
      super.testRandomHandPoses();
   }

   @Override
   public void testSingleSupport() throws Exception
   {
      super.testSingleSupport();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return getRobotModel().getSimpleRobotName();
   }
}
