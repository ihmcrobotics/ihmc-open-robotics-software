package us.ihmc.atlas.behaviorTests;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCObjectWeightBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasObjectWeightBehaviorTest extends DRCObjectWeightBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasObjectWeightBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 14.6)
   @Test(timeout = 73000)
   public void testConstructorAndSetInput()
   {
      super.testConstructorAndSetInput();
   }

   @Override
   @Ignore("Needs to be reimplemented")
   @ContinuousIntegrationTest(estimatedDuration = 19.6)
   @Test(timeout = 98000)
   public void testSettingWeight() throws SimulationExceededMaximumTimeException
   {
      super.testSettingWeight();
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
