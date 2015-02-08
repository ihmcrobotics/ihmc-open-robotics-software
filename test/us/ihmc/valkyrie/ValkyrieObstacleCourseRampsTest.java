package us.ihmc.valkyrie;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.obstacleCourseTests.DRCObstacleCourseRampsTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooPlanType;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.BambooPlan;

@BambooPlan(planType = {BambooPlanType.Fast, BambooPlanType.Video})
public class ValkyrieObstacleCourseRampsTest extends DRCObstacleCourseRampsTest
{

   private final DRCRobotModel robotModel = new ValkyrieRobotModel(false, false);
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
   protected double getMaxRotationCorruption()
   {
      return 0.0;
   }

}
