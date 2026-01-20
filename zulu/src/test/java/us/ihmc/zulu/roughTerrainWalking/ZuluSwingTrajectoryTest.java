package us.ihmc.zulu.roughTerrainWalking;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.zulu.ZuluVersion;
import us.ihmc.zulu.ZuluRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.HumanoidSwingTrajectoryTest;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

@Tag("humanoid-rough-terrain-slow")
public class ZuluSwingTrajectoryTest extends HumanoidSwingTrajectoryTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ALEXANDER);
   }

   @Override
   @Test
   public void testMultipleHeightFootsteps()
   {
      super.testMultipleHeightFootsteps();
   }

   @Override
   @Test
   public void testNegativeSwingHeight()
   {
      super.testNegativeSwingHeight();
   }

   @Override
   @Test
   public void testReallyHighFootstep()
   {
      super.testReallyHighFootstep();
   }

   @Override
   @Test
   public void testSelfCollisionAvoidance()
   {
      super.testSelfCollisionAvoidance();
   }
}
