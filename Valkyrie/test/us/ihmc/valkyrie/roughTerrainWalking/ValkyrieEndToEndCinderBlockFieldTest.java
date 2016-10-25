package us.ihmc.valkyrie.roughTerrainWalking;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.darpaRoboticsChallenge.roughTerrainWalking.EndToEndCinderBlockFieldTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST, IntegrationCategory.VIDEO})
public class ValkyrieEndToEndCinderBlockFieldTest extends EndToEndCinderBlockFieldTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

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
}
