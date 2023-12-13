package us.ihmc.atlas.networkProcessor.kinematicsToolboxModule;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;

public class AtlasHumanoidKinematicsToolboxControllerTest extends HumanoidKinematicsToolboxControllerTest
{
   private DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   private DRCRobotModel ghostRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @Tag("humanoid-toolbox")
   @Test // (timeout = 30000)
   public void testHoldBodyPose() throws Exception
   {
      super.testHoldBodyPose();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test // (timeout = 30000)
   public void testRandomHandPositions() throws Exception
   {
      super.testRandomHandPositions();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test // (timeout = 30000)
   public void testRandomHandPoses() throws Exception
   {
      super.testRandomHandPoses();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test // (timeout = 30000)
   public void testSingleSupport() throws Exception
   {
      super.testSingleSupport();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test
   public void testCenterOfMassConstraint() throws Exception
   {
      super.testCenterOfMassConstraint();
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
