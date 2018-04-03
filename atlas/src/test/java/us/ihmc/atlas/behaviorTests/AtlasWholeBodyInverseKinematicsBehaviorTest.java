package us.ihmc.atlas.behaviorTests;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.WholeBodyInverseKinematicsBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasWholeBodyInverseKinematicsBehaviorTest extends WholeBodyInverseKinematicsBehaviorTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForAHandPose() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForAHandPose();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForBothHandPoses() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForBothHandPoses();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 40000)
   public void testSolvingForChestAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForChestAngularControl();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForHandAngularLinearControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandAngularLinearControl();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForHandRollConstraint() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandRollConstraint();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForHandSelectionMatrix() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForHandSelectionMatrix();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 120000)
   public void testSolvingForPelvisAngularControl() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testSolvingForPelvisAngularControl();
   }
}
