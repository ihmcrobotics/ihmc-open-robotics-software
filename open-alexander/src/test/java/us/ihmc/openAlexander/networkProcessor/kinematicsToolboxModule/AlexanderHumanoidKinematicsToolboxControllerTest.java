package us.ihmc.openAlexander.networkProcessor.kinematicsToolboxModule;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.openAlexander.ZuluVersion;
import us.ihmc.openAlexander.OpenAlexanderRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxControllerTest;

public class AlexanderHumanoidKinematicsToolboxControllerTest extends HumanoidKinematicsToolboxControllerTest
{
   private DRCRobotModel robotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   private DRCRobotModel ghostRobotModel = new OpenAlexanderRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   @Tag("humanoid-toolbox")
   @Test
   public void testHoldBodyPose() throws Exception
   {
      super.testHoldBodyPose();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test
   public void testRandomHandPositions() throws Exception
   {
      super.testRandomHandPositions();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test
   public void testRandomHandPoses() throws Exception
   {
      super.testRandomHandPoses();
   }

   @Override
   @Tag("humanoid-toolbox")
   @Test
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
