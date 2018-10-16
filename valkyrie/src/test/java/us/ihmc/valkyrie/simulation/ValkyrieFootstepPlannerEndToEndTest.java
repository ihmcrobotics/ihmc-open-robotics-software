package us.ihmc.valkyrie.simulation;

import org.junit.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarBipedalFootstepPlannerEndToEndTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieFootstepPlannerEndToEndTest extends AvatarBipedalFootstepPlannerEndToEndTest
{
   private DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Override
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 120000)
   public void testWalkingBetweenBollardsAStarPlanner()
   {
      super.testWalkingBetweenBollardsAStarPlanner();
   }

}
