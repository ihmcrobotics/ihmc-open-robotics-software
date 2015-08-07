package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseTrialsTerrainTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;

@BambooPlan(planType = {BambooPlanType.Slow, BambooPlanType.VideoB})
public class ValkyrieObstacleCourseTrialsTerrainTest extends DRCObstacleCourseTrialsTerrainTest
{

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(false, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

}
