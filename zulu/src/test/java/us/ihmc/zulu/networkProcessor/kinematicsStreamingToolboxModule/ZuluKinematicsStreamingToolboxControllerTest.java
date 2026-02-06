package us.ihmc.zulu.networkProcessor.kinematicsStreamingToolboxModule;

import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;

public class ZuluKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT);
   }

   @Override
   @Test
   public void testStreamingToController()
   {
      super.testStreamingToController();
   }

   @Override
   @Test
   public void testHandMotionWithCollision()
   {
      super.testHandMotionWithCollision();
   }
}
