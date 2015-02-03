package us.ihmc.valkyrie.networkProcessor.depthData;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.DepthDataProcessorTest;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJob;
import us.ihmc.utilities.code.unitTesting.BambooAnnotations.CustomJobType;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@CustomJob(job = CustomJobType.Utilities)
public class ValkyrieDepthDataProcessorTest extends DepthDataProcessorTest
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
