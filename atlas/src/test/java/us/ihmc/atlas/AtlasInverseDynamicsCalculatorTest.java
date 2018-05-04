package us.ihmc.atlas;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCInverseDynamicsCalculatorTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

public class AtlasInverseDynamicsCalculatorTest extends DRCInverseDynamicsCalculatorTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics() throws UnreasonableAccelerationException
   {
      super.testInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testInverseDynamicsStartingWithRandomTorquesInSCS() throws UnreasonableAccelerationException
   {
      super.testInverseDynamicsStartingWithRandomTorquesInSCS();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
}
