package us.ihmc.valkyrie.networkProcessor.depthData;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiverTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.tools.agileTesting.BambooAnnotations.BambooPlan;
import us.ihmc.tools.agileTesting.BambooPlanType;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@BambooPlan(planType = BambooPlanType.Exclude)
public class ValkyriePointCloudDataReceiverTest extends PointCloudDataReceiverTest
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
}
