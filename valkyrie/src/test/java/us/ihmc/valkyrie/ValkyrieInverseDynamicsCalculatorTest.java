package us.ihmc.valkyrie;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.obstacleCourseTests.DRCInverseDynamicsCalculatorTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.EXCLUDE})
public class ValkyrieInverseDynamicsCalculatorTest extends DRCInverseDynamicsCalculatorTest
{
   private final DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics() throws UnreasonableAccelerationException
   {
      super.testInverseDynamicsStartingWithRandomAccelerationsInInverseDynamics();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
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
