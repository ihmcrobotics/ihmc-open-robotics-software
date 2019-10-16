package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxControllerTest;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsStreamingToolboxControllerTest extends KinematicsStreamingToolboxControllerTest
{
   @Override
   public DRCRobotModel newRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS, false);
   }

   @Override
   @Test
   public void testHandMotionWithCollision()
   {
      super.testHandMotionWithCollision();
   }
}
