package us.ihmc.valkyrie.simulation;

import java.io.InputStream;

import org.junit.jupiter.api.Test;

import us.ihmc.avatar.AvatarFlatGroundFastWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieFlatGroundFastWalkingTest extends AvatarFlatGroundFastWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS)
      {
         @Override
         public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
         {
            return super.createHumanoidFloatingRootJointRobot(createCollisionMeshes, false);
         }
         @Override
         public InputStream getParameterOverwrites()
         {
            return ValkyrieFlatGroundFastWalkingTest.class.getResourceAsStream("fast_walking_parameters.xml");
         }
      };
   }

   @Override
   public double getFastSwingTime()
   {
      return 0.50;
   }

   @Override
   public double getFastTransferTime()
   {
      return 0.05;
   }

   @Test
   @Override
   public void testForwardWalking() throws Exception
   {
      super.testForwardWalking();
   }
}
