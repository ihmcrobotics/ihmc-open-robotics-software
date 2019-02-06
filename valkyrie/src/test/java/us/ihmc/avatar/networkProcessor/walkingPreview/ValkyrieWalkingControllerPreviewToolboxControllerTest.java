package us.ihmc.avatar.networkProcessor.walkingPreview;

import org.junit.Test;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieWalkingControllerPreviewToolboxControllerTest extends AvatarWalkingControllerPreviewToolboxControllerTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
   private final ValkyrieRobotModel ghostRobotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 15.0)
   @Test(timeout = 300000)
   public void testWalkingPreviewAlone() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      super.testWalkingPreviewAlone();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 44.0)
   @Test(timeout = 300000)
   public void testStepsInPlacePreviewAtControllerDT() throws SimulationExceededMaximumTimeException
   {
      super.testStepsInPlacePreviewAtControllerDT();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 42.0)
   @Test(timeout = 300000)
   public void testResetFeature() throws SimulationExceededMaximumTimeException
   {
      super.testResetFeature();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public DRCRobotModel getGhostRobotModel()
   {
      return ghostRobotModel;
   }
}
